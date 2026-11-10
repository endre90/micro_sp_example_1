//! The simulated cell: the dice, the gripper, the camera, and the operator.
//!
//! One loop, so a gripper command and a disturbance can never interleave:
//!
//! * **Gripper** - serves the `gripper_request_*` channel. `grip` closes on a
//!   die only if that die is really between the fingers (for the die `unseen`,
//!   on whichever die is); `place` sets it down on whatever is below (and if
//!   that is not where it was meant to go - the tower was knocked over in the
//!   meantime - it says so); `throw` lets it tumble into the arena and land
//!   wherever, showing whatever. `look` finds a die for the build SOP and says
//!   whether it still has to be turned; `plan_flip` works out how to grip it so
//!   that tipping it onto the flip station brings the wanted face up (or one
//!   step closer to it), and publishes where.
//!
//! A die keeps its whole orientation, not just its top face: it is gripped as it
//! lies, turns with the gripper, and lands however the gripper was turned when it
//! let go. That is what makes a flip come out right.
//! * **Camera** - after every change, the `*_measured` variables. It looks at
//!   the arena only: a die from the feeder gets its colour (in RViz too) when
//!   it lands there.
//! * **Feeder** - publishes `feeder_next`, where the next die in it is. Which
//!   die that is, nobody knows.
//! * **Operator** - the `disturbance` variable: knock the tower over, take a
//!   die away, throw one, reset the cell.
//!
//! Anything the robot did not ask for raises `world_changed`, which is the
//! model's cue to look again and replan (model/reactions.rs).

pub mod world;

use std::sync::Arc;
use std::time::Duration;

use micro_sp::*;
use nalgebra::{Isometry3, Unit, UnitQuaternion, Vector3};
use rand::Rng;

use crate::model::state::NONE;
use crate::*;
use world::*;

const T: &str = "cell";

/// How close the die has to be to the TCP for a grip to close on it.
const GRIP_TOLERANCE: f64 = 0.02;
const ANIMATION_STEPS: usize = 12;
const ANIMATION_STEP_MS: u64 = 25;

pub struct Cell {
    pub world: World,
    pub con: SPConnection,
}

enum Outcome {
    Succeeded(String),
    Failed(String),
}

impl Cell {
    pub async fn new(world: World, connection_manager: &Arc<ConnectionManager>) -> Cell {
        let con = connection_manager.get_connection().await;
        Cell { world, con }
    }

    /// Puts every die where the world says it is and tells the model to look.
    pub async fn publish_all(&mut self) {
        let frames = self.world.all_frames();
        if let Err(e) = TransformsManager::insert_transforms(&mut self.con, &frames).await {
            log::error!(target: T, "Could not write the dice frames: {e}");
        }
        let names: Vec<String> = self.world.dice.iter().map(|d| d.name.clone()).collect();
        for name in names {
            self.publish_transit_of(&name).await;
        }
        self.publish_measured(true).await;
        StateManager::set_sp_value(&mut self.con, "cell_online", &true.to_spvalue()).await;
    }

    async fn publish_frames_of(&mut self, name: &str) {
        let Some(die) = self.world.die(name).cloned() else { return };
        let frames = vec![self.world.pose_frame(&die), self.world.resting_body_frame(&die), self.world.pips_frame(&die)];
        if let Err(e) = TransformsManager::insert_transforms(&mut self.con, &frames).await {
            log::error!(target: T, "Could not move {name}: {e}");
        }
        self.publish_transit_of(name).await;
    }

    /// The point above a resting die that the robot arrives and leaves through.
    /// Left alone while the die is in the gripper, so the lift after a grip goes
    /// back up to where the die was picked from.
    async fn publish_transit_of(&mut self, name: &str) {
        if self.world.die(name).map(|d| d.loc == Loc::Gripper).unwrap_or(true) {
            return;
        }
        let frame = die_frame(name);
        if let Some(pose) = self.world_pose(&frame).await {
            self.publish_transit(&frame, &pose).await;
        }
    }

    /// `{frame}_transit`: at SAFE_Z above `pose`, turned like it about the vertical.
    async fn publish_transit(&mut self, frame: &str, pose: &Isometry3<f64>) {
        let heading = pose.rotation.euler_angles().2;
        let at = Vector3::new(pose.translation.x, pose.translation.y, SAFE_Z);
        let transit = stamped(&transit_frame(frame), WORLD_FRAME, iso(at, yaw(heading)), MapOrUnknown::UNKNOWN, false);
        if let Err(e) = TransformsManager::insert_transform(&mut self.con, &transit).await {
            log::error!(target: T, "Could not write {}: {e}", transit.child_frame_id);
        }
    }

    async fn world_pose(&mut self, frame: &str) -> Option<Isometry3<f64>> {
        match TransformsManager::lookup_transform(&mut self.con, WORLD_FRAME, frame).await {
            Ok(tf) => Some(sp_transform_to_isometry(tf.transform)),
            Err(e) => {
                log::warn!(target: T, "Cannot locate {frame}: {e}");
                None
            }
        }
    }

    /// The camera's reading. `surprise` raises `world_changed` with it.
    async fn publish_measured(&mut self, surprise: bool) {
        let mut state = self.world.measured_state();
        if surprise {
            state.add_mut(SPAssignment::new(SPVariable::new("world_changed", SPValueType::Bool), true.to_spvalue()), T);
        }
        StateManager::set_state(&mut self.con, &state).await;
        self.publish_feeder_next().await;
    }

    /// `feeder_next` and its transit: where the robot picks the next die from the
    /// feeder. Left alone while the gripper holds a die, so the lift after a grip
    /// in the feeder goes back up from the slot it was in.
    async fn publish_feeder_next(&mut self) {
        if self.world.holding().is_some() {
            return;
        }
        let Some(next) = self.world.next_in_feeder().map(|d| d.name.clone()) else { return };
        let Some(pose) = self.world_pose(&die_frame(&next)).await else { return };
        let frame = stamped(FEEDER_NEXT_FRAME, WORLD_FRAME, pose, MapOrUnknown::UNKNOWN, false);
        if let Err(e) = TransformsManager::insert_transform(&mut self.con, &frame).await {
            log::error!(target: T, "Could not write {FEEDER_NEXT_FRAME}: {e}");
        }
        self.publish_transit(FEEDER_NEXT_FRAME, &pose).await;
    }

    async fn wanted_face(&mut self, name: &str) -> Option<i64> {
        match StateManager::get_sp_value(&mut self.con, &format!("{name}_wanted_face")).await {
            Some(SPValue::Int64(IntOrUnknown::Int64(face))) if (1..=6).contains(&face) => Some(face),
            _ => None,
        }
    }

    /// Tells the build SOP whether `name` still has to be turned, and where to
    /// pick it up for the tower: `build_pick`, where it lies now.
    async fn publish_build_pick(&mut self, name: &str) -> Result<bool, String> {
        let die = self.world.die(name).cloned().ok_or(format!("there is no {name} die"))?;
        if die.loc == Loc::Gripper {
            return Err(format!("{name} is in the gripper"));
        }
        let wanted = self.wanted_face(name).await.ok_or(format!("{name} is not in the order"))?;
        let pose = self.world_pose(&die_frame(name)).await.ok_or(format!("cannot see {name}"))?;
        let frame = stamped(BUILD_PICK_FRAME, WORLD_FRAME, pose, MapOrUnknown::UNKNOWN, false);
        let _ = TransformsManager::insert_transform(&mut self.con, &frame).await;
        self.publish_transit(BUILD_PICK_FRAME, &pose).await;
        let flip_needed = die.face() != wanted;
        StateManager::set_sp_value(&mut self.con, "gripper_flip_needed", &flip_needed.to_spvalue()).await;
        Ok(flip_needed)
    }

    async fn world_position(&mut self, frame: &str) -> Option<Vector3<f64>> {
        match TransformsManager::lookup_transform(&mut self.con, WORLD_FRAME, frame).await {
            Ok(tf) => {
                let t = tf.transform.translation;
                Some(Vector3::new(*t.x, *t.y, *t.z))
            }
            Err(e) => {
                log::warn!(target: T, "Cannot locate {frame}: {e}");
                None
            }
        }
    }

    // --- Gripper ----------------------------------------------------------------

    /// The die nearest the TCP, for a grip on "whichever die is there".
    async fn die_at_fingers(&mut self) -> Option<String> {
        let tcp = self.world_position(TCP_FRAME).await?;
        let mut best: Option<(String, f64)> = None;
        let names: Vec<String> = self.world.dice.iter().map(|d| d.name.clone()).collect();
        for name in names {
            if let Some(at) = self.world_position(&die_frame(&name)).await {
                let distance = (tcp - at).norm();
                if best.as_ref().is_none_or(|(_, d)| distance < *d) {
                    best = Some((name, distance));
                }
            }
        }
        best.map(|(name, _)| name)
    }

    async fn grip(&mut self, name: &str) -> Outcome {
        if let Some(held) = self.world.holding() {
            return Outcome::Failed(format!("already holding {}", held.name));
        }
        let name = if name == UNKNOWN_DIE {
            match self.die_at_fingers().await {
                Some(name) => name,
                None => return Outcome::Failed("closed on nothing".to_string()),
            }
        } else {
            name.to_string()
        };
        let name = name.as_str();
        let Some(die) = self.world.die(name).cloned() else {
            return Outcome::Failed(format!("there is no {name} die"));
        };
        let (Some(tcp), Some(at)) = (self.world_position(TCP_FRAME).await, self.world_position(&die_frame(name)).await) else {
            return Outcome::Failed("cannot see the gripper or the die".to_string());
        };
        let distance = (tcp - at).norm();
        if distance > GRIP_TOLERANCE {
            return Outcome::Failed(format!(
                "closed on nothing - {name} is {:.0} mm from the fingers",
                distance * 1000.0
            ));
        }
        // The die keeps how it lies; from now on that is relative to the TCP.
        let (Some(tcp_pose), Some(body_pose)) = (self.world_pose(TCP_FRAME).await, self.world_pose(&die_body_frame(name)).await) else {
            return Outcome::Failed("cannot see the gripper or the die".to_string());
        };
        let held = snap_to_aligned(&(tcp_pose.rotation.inverse() * body_pose.rotation));
        let was = die.loc.clone();
        let what = if die.seen { name.to_string() } else { "a die".to_string() };
        let die = self.world.die_mut(name).unwrap();
        die.loc = Loc::Gripper;
        die.yaw = 0.0;
        die.body = held;
        self.publish_frames_of(name).await;
        self.publish_measured(false).await;
        Outcome::Succeeded(format!("gripped {what} ({})", describe_loc(&was)))
    }

    /// The held die, checked against what the request names.
    fn held_as(&self, name: &str) -> Result<String, String> {
        match self.world.holding() {
            Some(d) if name == UNKNOWN_DIE || d.name == name => Ok(d.name.clone()),
            _ => Err(format!("not holding {name}")),
        }
    }

    async fn place(&mut self, name: &str, target: &str) -> Outcome {
        let name = match self.held_as(name) {
            Ok(n) => n,
            Err(e) => return Outcome::Failed(e),
        };
        let name = name.as_str();
        let Some(tcp_pose) = self.world_pose(TCP_FRAME).await else {
            return Outcome::Failed("cannot see the gripper".to_string());
        };
        let (landed, surprise) = if target == FLIP_STATION_FRAME {
            if self.world.on_station().is_some() {
                let (x, y) = self.world.free_arena_spot();
                (Loc::Arena(x, y), true)
            } else {
                (Loc::Station, false)
            }
        } else if let Some(k) = target.strip_prefix("level_").and_then(|k| k.parse::<usize>().ok()) {
            // Whatever is below it decides where a die ends up. If the tower
            // shrank while the robot was on its way, the die drops onto what is left.
            let landed = self.world.tower_height() + 1;
            (Loc::Level(landed.min(k)), landed < k)
        } else if let Some(j) = target.strip_prefix("pad_").and_then(|j| j.parse::<usize>().ok()) {
            if self.world.at_pad(j).is_some() {
                let (x, y) = self.world.free_arena_spot();
                (Loc::Arena(x, y), true)
            } else {
                (Loc::Pad(j), false)
            }
        } else {
            return Outcome::Failed(format!("do not know how to place on '{target}'"));
        };
        // How it lies now is how the gripper held it, seen from where it landed.
        let held = self.world.die(name).unwrap().body;
        let die = self.world.die_mut(name).unwrap();
        die.loc = landed.clone();
        die.yaw = 0.0;
        let pose = self.world.pose_frame(self.world.die(name).unwrap());
        let _ = TransformsManager::insert_transform(&mut self.con, &pose).await;
        if let Some(resting) = self.world_pose(&die_frame(name)).await {
            self.world.die_mut(name).unwrap().body = snap_to_aligned(&(resting.rotation.inverse() * tcp_pose.rotation * held));
        }
        self.publish_frames_of(name).await;
        self.publish_measured(surprise).await;
        if surprise {
            log::warn!(target: T, "{name} was put on {target} but ended up {}.", describe_loc(&landed));
        }
        if landed == Loc::Station {
            // A flip: say whether it was enough, and where to pick the die up.
            let flips = match StateManager::get_sp_value(&mut self.con, "kpi_run_flips").await {
                Some(SPValue::Int64(IntOrUnknown::Int64(n))) => n,
                _ => 0,
            };
            StateManager::set_sp_value(&mut self.con, "kpi_run_flips", &(flips + 1).to_spvalue()).await;
            let face = self.world.die(name).unwrap().face();
            return match self.publish_build_pick(name).await {
                Ok(true) => Outcome::Succeeded(format!("flipped {name} to {face}, one more to go")),
                Ok(false) => Outcome::Succeeded(format!("flipped {name} to {face}")),
                Err(e) => Outcome::Failed(e),
            };
        }
        Outcome::Succeeded(format!("placed {name} {}", describe_loc(&landed)))
    }

    async fn look(&mut self, name: &str) -> Outcome {
        let face = self.world.die(name).map(|d| d.face()).unwrap_or(0);
        match self.publish_build_pick(name).await {
            Ok(true) => Outcome::Succeeded(format!("{name} shows {face}: to be turned")),
            Ok(false) => Outcome::Succeeded(format!("{name} shows {face}: as wanted")),
            Err(e) => Outcome::Failed(e),
        }
    }

    async fn throw(&mut self, name: &str) -> Outcome {
        let name = match self.held_as(name) {
            Ok(n) => n,
            Err(e) => return Outcome::Failed(e),
        };
        self.tumble_into_arena(std::slice::from_ref(&name)).await;
        let face = self.world.die(&name).map(|d| d.face()).unwrap_or(0);
        self.publish_measured(false).await;
        Outcome::Succeeded(format!("threw a die: the camera sees {name}, showing {face}"))
    }

    /// Plans one flip of `name` towards `wanted`: which face to bring up next,
    /// and how to grip the die so that face comes up when it is tipped over.
    ///
    /// The tip turns the die a quarter about the fingers' axis, and brings up the
    /// face that was towards the TCP's +y (see `flip_station_tilt` in the scene).
    /// So: the wanted face if it is on a side, otherwise any side face - the
    /// bottom one takes two flips. Then the grip is the die's own frame turned
    /// about the vertical, in quarter steps, until that face is towards +y.
    async fn plan_flip(&mut self, name: &str, wanted: i64) -> Outcome {
        let Some(die) = self.world.die(name).cloned() else {
            return Outcome::Failed(format!("there is no {name} die"));
        };
        if die.loc == Loc::Gripper {
            return Outcome::Failed(format!("{name} is in the gripper"));
        }
        if !(1..=6).contains(&wanted) {
            return Outcome::Failed(format!("{wanted} is not a face"));
        }
        let top = die.face();
        if top == wanted {
            return Outcome::Failed(format!("{name} already shows {wanted}"));
        }
        let Some(pose) = self.world_pose(&die_frame(name)).await else {
            return Outcome::Failed(format!("cannot see {name}"));
        };
        let quarter_for = |face: i64| -> f64 {
            let n = die.body * face_normal(face);
            let delta = (-n.x).atan2(n.y);
            (delta / std::f64::consts::FRAC_PI_2).round() * std::f64::consts::FRAC_PI_2
        };
        let sides: Vec<i64> = (1..=6).filter(|f| *f != top && *f != 7 - top).collect();
        let next = if wanted != 7 - top {
            wanted
        } else {
            // Two flips either way; take the side that needs the least wrist.
            let heading = pose.rotation.euler_angles().2;
            *sides
                .iter()
                .min_by(|a, b| {
                    let turn = |f: i64| (heading + quarter_for(f)).sin().atan2((heading + quarter_for(f)).cos()).abs();
                    turn(**a).total_cmp(&turn(**b))
                })
                .unwrap()
        };
        let grip = iso(pose.translation.vector, pose.rotation * yaw(quarter_for(next)));
        let grip_frame = stamped(FLIP_GRIP_FRAME, WORLD_FRAME, grip, MapOrUnknown::UNKNOWN, false);
        let _ = TransformsManager::insert_transform(&mut self.con, &grip_frame).await;
        self.publish_transit(FLIP_GRIP_FRAME, &grip).await;

        let steps = if wanted == 7 - top { "two flips" } else { "one flip" };
        Outcome::Succeeded(format!("{name} shows {top}; bringing {next} up ({steps} to {wanted})"))
    }

    /// Dice falling into the arena: each lands on a free spot, yawed and showing
    /// a face at random, and gets there with a bit of a tumble on the way. The
    /// camera reads each one as it lands - which is when a die from the feeder
    /// gets its colour.
    async fn tumble_into_arena(&mut self, names: &[String]) {
        struct Flight {
            name: String,
            from: nalgebra::Isometry3<f64>,
            body_from: UnitQuaternion<f64>,
            to: (f64, f64, f64),
            body: UnitQuaternion<f64>,
            spin_axis: Unit<Vector3<f64>>,
        }
        let mut flights = vec![];
        for name in names {
            // Where it is now, seen from the arena, so the fall starts there.
            let from = match TransformsManager::lookup_transform(&mut self.con, ARENA_FRAME, &die_frame(name)).await {
                Ok(tf) => sp_transform_to_isometry(tf.transform),
                Err(_) => nalgebra::Isometry3::translation(0.0, 0.0, 0.15),
            };
            let body_from = TransformsManager::get_transform(&mut self.con, &die_body_frame(name))
                .await
                .map(|tf| sp_transform_to_isometry(tf.transform).rotation)
                .unwrap_or_else(|_| UnitQuaternion::identity());
            let (x, y) = self.world.free_arena_spot();
            let yaw = self.world.random_yaw();
            let body = self.world.random_body();
            let spin_axis = Unit::new_normalize(Vector3::new(
                self.world.rng.gen_range(-1.0..1.0),
                self.world.rng.gen_range(-1.0..1.0),
                self.world.rng.gen_range(-0.3..0.3),
            ));
            // Claim the spot now, so the next die in the same fall picks another.
            let die = self.world.die_mut(name).unwrap();
            die.loc = Loc::Arena(x, y);
            die.yaw = yaw;
            die.body = body;
            flights.push(Flight { name: name.clone(), from, body_from, to: (x, y, yaw), body, spin_axis });
        }

        for step in 1..=ANIMATION_STEPS {
            let t = step as f64 / ANIMATION_STEPS as f64;
            let mut frames = vec![];
            for f in &flights {
                let Some(die) = self.world.die(&f.name).cloned() else { continue };
                let start = f.from.translation.vector;
                let end = Vector3::new(f.to.0, f.to.1, ARENA_DIE_Z);
                let mut position = start.lerp(&end, t);
                position.z += 0.05 * (std::f64::consts::PI * t).sin() * (1.0 - t);
                let rotation = f.from.rotation.slerp(&yaw(f.to.2), t);
                let spin = UnitQuaternion::from_axis_angle(&f.spin_axis, 3.0 * std::f64::consts::TAU * (1.0 - t));
                let body = spin * f.body_from.slerp(&f.body, t);
                let mut pose = self.world.pose_frame(&die);
                pose.parent_frame_id = ARENA_FRAME.to_string();
                pose.transform = isometry_to_sp_transform(iso(position, rotation));
                frames.push(pose);
                frames.push(self.world.body_frame(&die, body));
            }
            let _ = TransformsManager::insert_transforms(&mut self.con, &frames).await;
            tokio::time::sleep(Duration::from_millis(ANIMATION_STEP_MS)).await;
        }
        for f in &flights {
            if let Some(die) = self.world.die_mut(&f.name) {
                die.seen = true;
            }
            self.publish_frames_of(&f.name).await;
        }
    }

    // --- Operator -----------------------------------------------------------------

    /// The dice from level `k` up, top first.
    fn tower_from(&self, k: usize) -> Vec<String> {
        (k..=self.world.tower_height()).rev().filter_map(|l| self.world.at_level(l).map(|d| d.name.clone())).collect()
    }

    async fn to_feeder(&mut self, name: &str) -> Result<String, String> {
        let slot = self.world.free_feeder_slot().ok_or("the feeder is full")?;
        let die = self.world.die_mut(name).unwrap();
        die.loc = Loc::Feeder(slot);
        die.yaw = 0.0;
        // Out of sight: from now on it is just a die in the feeder.
        die.seen = false;
        self.publish_frames_of(name).await;
        Ok(format!("{name} put back in feeder slot {slot}, still showing {}", self.world.die(name).unwrap().face()))
    }

    /// Takes the dice above `name` off with it, if it is in the middle of the tower.
    async fn knock_down_above(&mut self, name: &str) {
        if let Some(Loc::Level(k)) = self.world.die(name).map(|d| d.loc.clone()) {
            let above: Vec<String> = self.tower_from(k + 1);
            if !above.is_empty() {
                self.tumble_into_arena(&above).await;
            }
        }
    }

    async fn disturb(&mut self, command: &str) -> Result<String, String> {
        let mut words = command.split_whitespace();
        let verb = words.next().unwrap_or("").to_ascii_lowercase();
        let arg = words.next().map(|w| w.to_ascii_lowercase());
        let held = self.world.holding().map(|d| d.name.clone());
        let check = |world: &World, name: &str| -> Result<(), String> {
            if world.die(name).is_none() {
                return Err(format!("there is no {name} die"));
            }
            if held.as_deref() == Some(name) {
                return Err(format!("{name} is in the gripper - try again in a moment"));
            }
            Ok(())
        };

        match (verb.as_str(), arg) {
            ("knock_tower", _) | ("knock", _) => {
                let tower = self.tower_from(1);
                if tower.is_empty() {
                    return Err("there is no tower to knock over".to_string());
                }
                self.tumble_into_arena(&tower).await;
                Ok(format!("knocked the tower over - {} dice in the arena", tower.len()))
            }
            ("steal_top", _) => {
                let top = self.world.tower_height();
                let name = self.world.at_level(top).map(|d| d.name.clone()).ok_or("the tower is empty")?;
                self.to_feeder(&name).await
            }
            ("steal", Some(name)) => {
                check(&self.world, &name)?;
                self.knock_down_above(&name).await;
                self.to_feeder(&name).await
            }
            ("bump", Some(name)) | ("throw", Some(name)) => {
                check(&self.world, &name)?;
                self.knock_down_above(&name).await;
                self.tumble_into_arena(&[name.clone()]).await;
                Ok(format!("threw {name} into the arena: it shows {}", self.world.die(&name).unwrap().face()))
            }
            ("reset", _) => {
                let seed = self.world.rng.r#gen();
                let mut fresh = World::new(self.world.config.clone(), seed);
                if let Some(name) = &held {
                    // Leave the held die alone; everything else starts over.
                    let held_die = self.world.die(name).cloned().unwrap();
                    *fresh.die_mut(name).unwrap() = held_die;
                }
                self.world.dice = fresh.dice;
                let frames = self.world.all_frames();
                let _ = TransformsManager::insert_transforms(&mut self.con, &frames).await;
                Ok("every die back in the feeder, in a new order".to_string())
            }
            _ => Err(format!(
                "unknown disturbance '{command}' - try knock_tower, steal_top, steal <colour>, bump <colour>, reset"
            )),
        }
    }
}

fn describe_loc(loc: &Loc) -> String {
    match loc {
        Loc::Feeder(s) => format!("from feeder slot {s}"),
        Loc::Arena(..) => "in the arena".to_string(),
        Loc::Pad(j) => format!("on pad {j}"),
        Loc::Level(k) => format!("on level {k}"),
        Loc::Station => "on the flip station".to_string(),
        Loc::Gripper => "in the gripper".to_string(),
    }
}

fn put(state: &mut State, name: &str, value: SPValue) {
    state.add_mut(SPAssignment::new(SPVariable::new(name, value.has_type()), value), T);
}

/// The cell's loop: one gripper request or one disturbance at a time.
pub async fn run_cell(mut cell: Cell) {
    let keys: Vec<String> = [
        "gripper_request_trigger",
        "gripper_request_state",
        "gripper_command",
        "gripper_die",
        "gripper_target",
        "disturbance",
        "kpi_run_disturbances",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    let mut ticker = tokio::time::interval(Duration::from_millis(10));

    loop {
        ticker.tick().await;
        let Some(state) = StateManager::get_state_for_keys(&mut cell.con, &keys, T).await else {
            continue;
        };

        let trigger = state.get_bool_or_default_to_false("gripper_request_trigger", T);
        let request_state = state.get_string_or_default_to_unknown("gripper_request_state", T);
        if trigger && request_state == "initial" {
            let command = state.get_string_or_default_to_unknown("gripper_command", T);
            let die = state.get_string_or_default_to_unknown("gripper_die", T);
            let target = state.get_string_or_default_to_unknown("gripper_target", T);
            StateManager::set_sp_value(&mut cell.con, "gripper_request_state", &"executing".to_spvalue()).await;

            let outcome = match command.as_str() {
                "grip" => cell.grip(&die).await,
                "place" => cell.place(&die, &target).await,
                "throw" => cell.throw(&die).await,
                "look" => cell.look(&die).await,
                "plan_flip" => {
                    // The face to flip towards is the order's, for this die.
                    match cell.wanted_face(&die).await {
                        Some(face) => cell.plan_flip(&die, face).await,
                        None => Outcome::Failed(format!("{die} is not in the order")),
                    }
                }
                other => Outcome::Failed(format!("unknown gripper command '{other}'")),
            };
            let mut reply = State::new();
            match outcome {
                Outcome::Succeeded(result) => {
                    log::info!(target: T, "Gripper: {result}.");
                    put(&mut reply, "gripper_result", result.to_spvalue());
                    put(&mut reply, "gripper_request_state", "succeeded".to_spvalue());
                }
                Outcome::Failed(reason) => {
                    log::warn!(target: T, "Gripper {command} {die} failed: {reason}.");
                    put(&mut reply, "gripper_result", reason.to_spvalue());
                    put(&mut reply, "gripper_request_state", "failed".to_spvalue());
                }
            }
            StateManager::set_state(&mut cell.con, &reply).await;
            continue;
        }

        let disturbance = state.get_string_or_value("disturbance", String::new(), T);
        if !disturbance.trim().is_empty() && request_state != "executing" {
            let result = cell.disturb(disturbance.trim()).await;
            cell.publish_feeder_next().await;
            let mut reply = cell.world.measured_state();
            put(&mut reply, "disturbance", "".to_spvalue());
            match result {
                Ok(what) => {
                    log::warn!(target: T, "Disturbance: {what}.");
                    put(&mut reply, "disturbance_status", what.to_spvalue());
                    put(&mut reply, "world_changed", true.to_spvalue());
                    let count = state.get_int_or_default_to_zero("kpi_run_disturbances", T) + 1;
                    put(&mut reply, "kpi_run_disturbances", count.to_spvalue());
                }
                Err(why) => {
                    log::warn!(target: T, "Disturbance '{disturbance}' not applied: {why}.");
                    put(&mut reply, "disturbance_status", format!("not applied: {why}").to_spvalue());
                }
            }
            StateManager::set_state(&mut cell.con, &reply).await;
        }
    }
}

/// Clears the gripper channel and what the gripper holds, for a fresh start.
pub async fn reset_gripper(con: &mut SPConnection) {
    let mut state = State::new();
    put(&mut state, "gripper_request_state", "initial".to_spvalue());
    put(&mut state, "gripper_request_trigger", false.to_spvalue());
    put(&mut state, "gripper_holding", NONE.to_spvalue());
    put(&mut state, "disturbance", "".to_spvalue());
    StateManager::set_state(con, &state).await;
}
