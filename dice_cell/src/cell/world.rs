//! The truth about the dice: where each one is and how it lies.
//!
//! The control system never reads this. It sees the cell through two windows,
//! both written from here: the transform tree (which RViz draws, and which the
//! robot's IK looks up) and the `*_measured` variables, which stand in for a
//! camera that reads every die's position and top face.

use micro_sp::*;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rand::rngs::SmallRng;
use rand::seq::SliceRandom;
use rand::{Rng, SeedableRng};
use std::time::SystemTime;

use crate::model::state::NONE;
use crate::*;

#[derive(Debug, Clone, PartialEq)]
pub enum Loc {
    Feeder(usize),
    /// Lying in the arena at (x, y) in the arena frame.
    Arena(f64, f64),
    Pad(usize),
    Level(usize),
    /// On the flip station's post - only ever for a moment, during a flip.
    Station,
    Gripper,
}

#[derive(Debug, Clone)]
pub struct Die {
    pub name: String,
    pub loc: Loc,
    /// Rotation about the vertical, for dice in the feeder and the arena.
    pub yaw: f64,
    /// How the die lies in its pose frame: one of the 24 square orientations.
    /// Which face is up follows from it, and so does which faces are on the
    /// sides - which is what a flip needs to know. In the gripper it is relative
    /// to the TCP.
    pub body: UnitQuaternion<f64>,
}

impl Die {
    /// The face on top. Meaningless while the die is in the gripper.
    pub fn face(&self) -> i64 {
        face_showing(&self.body)
    }
}

pub struct World {
    pub config: Config,
    pub dice: Vec<Die>,
    pub rng: SmallRng,
}

impl World {
    /// Every die in a feeder slot, in a random order, showing a random face.
    pub fn new(config: Config, seed: u64) -> World {
        let mut rng = SmallRng::seed_from_u64(seed);
        let mut slots: Vec<usize> = (1..=config.dice.len()).collect();
        slots.shuffle(&mut rng);
        let dice = config
            .dice
            .iter()
            .zip(slots)
            .map(|(name, slot)| Die {
                name: name.clone(),
                loc: Loc::Feeder(slot),
                yaw: rng.gen_range(-0.15..0.15),
                body: aligned_rotations()[rng.gen_range(0..24)],
            })
            .collect();
        World { config, dice, rng }
    }

    pub fn die(&self, name: &str) -> Option<&Die> {
        self.dice.iter().find(|d| d.name == name)
    }

    pub fn die_mut(&mut self, name: &str) -> Option<&mut Die> {
        self.dice.iter_mut().find(|d| d.name == name)
    }

    pub fn at_level(&self, k: usize) -> Option<&Die> {
        self.dice.iter().find(|d| d.loc == Loc::Level(k))
    }

    pub fn at_pad(&self, j: usize) -> Option<&Die> {
        self.dice.iter().find(|d| d.loc == Loc::Pad(j))
    }

    pub fn on_station(&self) -> Option<&Die> {
        self.dice.iter().find(|d| d.loc == Loc::Station)
    }

    pub fn holding(&self) -> Option<&Die> {
        self.dice.iter().find(|d| d.loc == Loc::Gripper)
    }

    /// The highest level that holds a die, 0 for an empty tower.
    pub fn tower_height(&self) -> usize {
        (1..=self.config.levels()).filter(|k| self.at_level(*k).is_some()).max().unwrap_or(0)
    }

    pub fn free_feeder_slot(&mut self) -> Option<usize> {
        let taken: Vec<usize> = self
            .dice
            .iter()
            .filter_map(|d| if let Loc::Feeder(s) = d.loc { Some(s) } else { None })
            .collect();
        (1..=MAX_DICE).find(|s| !taken.contains(s))
    }

    fn arena_clearance(&self, candidate: (f64, f64)) -> f64 {
        self.dice
            .iter()
            .filter_map(|d| if let Loc::Arena(x, y) = d.loc { Some((x, y)) } else { None })
            .map(|(x, y)| ((x - candidate.0).powi(2) + (y - candidate.1).powi(2)).sqrt())
            .fold(f64::MAX, f64::min)
    }

    /// Somewhere on the arena floor that no other die is lying on.
    pub fn free_arena_spot(&mut self) -> (f64, f64) {
        let reach = ARENA_INNER_HALF - ARENA_MARGIN;
        let mut best = (0.0, 0.0);
        let mut best_clearance = f64::MIN;
        for _ in 0..80 {
            let candidate = (self.rng.gen_range(-reach..reach), self.rng.gen_range(-reach..reach));
            let clearance = self.arena_clearance(candidate);
            if clearance > ARENA_SPACING {
                return candidate;
            }
            if clearance > best_clearance {
                best = candidate;
                best_clearance = clearance;
            }
        }
        best
    }

    /// The free spot on the arena floor nearest `(x, y)` (arena coordinates), so
    /// a flipped die does not travel further than it has to.
    pub fn free_arena_spot_near(&self, target: (f64, f64)) -> (f64, f64) {
        let reach = ARENA_INNER_HALF - ARENA_MARGIN;
        let steps = 12;
        let mut best: Option<((f64, f64), f64)> = None;
        let mut roomiest = ((0.0, 0.0), f64::MIN);
        for i in 0..=steps {
            for j in 0..=steps {
                let candidate = (-reach + 2.0 * reach * i as f64 / steps as f64, -reach + 2.0 * reach * j as f64 / steps as f64);
                let clearance = self.arena_clearance(candidate);
                if clearance > roomiest.1 {
                    roomiest = (candidate, clearance);
                }
                if clearance > ARENA_SPACING {
                    let distance = ((candidate.0 - target.0).powi(2) + (candidate.1 - target.1).powi(2)).sqrt();
                    if best.is_none_or(|(_, d)| distance < d) {
                        best = Some((candidate, distance));
                    }
                }
            }
        }
        best.map(|(c, _)| c).unwrap_or(roomiest.0)
    }

    /// How a thrown die lands: any of the 24 square orientations.
    pub fn random_body(&mut self) -> UnitQuaternion<f64> {
        aligned_rotations()[self.rng.gen_range(0..24)]
    }

    /// A die's top face lands random; its yaw lands anywhere in a quarter turn,
    /// which is all a square gripper ever needs to rotate to grip it.
    pub fn random_yaw(&mut self) -> f64 {
        self.rng.gen_range(-std::f64::consts::FRAC_PI_4..std::f64::consts::FRAC_PI_4)
    }

    // --- What the camera sees --------------------------------------------------

    pub fn measured_state(&self) -> State {
        let mut state = State::new();
        let mut put = |name: String, value: SPValue| {
            state.add_mut(SPAssignment::new(SPVariable::new(&name, value.has_type()), value), "cell");
        };
        for die in &self.dice {
            // The flip station stands by the arena, and a die is only ever on it
            // during a flip: as far as the model is concerned it is in the arena.
            let in_arena = matches!(die.loc, Loc::Arena(..) | Loc::Station);
            let loose = in_arena || matches!(die.loc, Loc::Feeder(_));
            put(format!("{}_loose_measured", die.name), loose.to_spvalue());
            put(format!("{}_in_arena_measured", die.name), in_arena.to_spvalue());
            put(format!("{}_face_measured", die.name), die.face().to_spvalue());
        }
        for k in 1..=self.config.levels() {
            let (name, face) = self.at_level(k).map(|d| (d.name.clone(), d.face())).unwrap_or((NONE.to_string(), 0));
            put(format!("level_{k}_measured"), name.to_spvalue());
            put(format!("level_{k}_face_measured"), face.to_spvalue());
        }
        for j in 1..=self.config.pads {
            let (name, face) = self.at_pad(j).map(|d| (d.name.clone(), d.face())).unwrap_or((NONE.to_string(), 0));
            put(format!("pad_{j}_measured"), name.to_spvalue());
            put(format!("pad_{j}_face_measured"), face.to_spvalue());
        }
        let holding = self.holding().map(|d| d.name.clone()).unwrap_or(NONE.to_string());
        put("gripper_holding".to_string(), holding.to_spvalue());
        state
    }

    // --- What RViz and the robot's IK see ---------------------------------------

    /// The die's pose frame, under whatever it is resting on.
    pub fn pose_frame(&self, die: &Die) -> SPTransformStamped {
        let (parent, iso) = match die.loc {
            Loc::Feeder(slot) => (feeder_slot_frame(slot), iso(Vector3::zeros(), yaw(die.yaw))),
            Loc::Arena(x, y) => (ARENA_FRAME.to_string(), iso(Vector3::new(x, y, ARENA_DIE_Z), yaw(die.yaw))),
            Loc::Pad(j) => (pad_frame(j), Isometry3::identity()),
            Loc::Level(k) => (level_frame(k), Isometry3::identity()),
            Loc::Station => (FLIP_STATION_FRAME.to_string(), Isometry3::identity()),
            Loc::Gripper => (TCP_FRAME.to_string(), Isometry3::identity()),
        };
        stamped(&die_frame(&die.name), &parent, iso, MapOrUnknown::UNKNOWN, true)
    }

    /// The body frame as the die lies now.
    pub fn resting_body_frame(&self, die: &Die) -> SPTransformStamped {
        self.body_frame(die, die.body)
    }

    pub fn body_frame(&self, die: &Die, rotation: UnitQuaternion<f64>) -> SPTransformStamped {
        let colour = die_colour(&die.name).map(|c| c.rgb).unwrap_or([0.8, 0.8, 0.8]);
        stamped(
            &die_body_frame(&die.name),
            &die_frame(&die.name),
            iso(Vector3::zeros(), rotation),
            mesh_metadata("die_body.stl", colour),
            true,
        )
    }

    pub fn pips_frame(&self, die: &Die) -> SPTransformStamped {
        let colour = die_colour(&die.name).map(|c| c.pips).unwrap_or([0.0, 0.0, 0.0]);
        stamped(
            &die_pips_frame(&die.name),
            &die_body_frame(&die.name),
            Isometry3::identity(),
            mesh_metadata("die_pips.stl", colour),
            false,
        )
    }

    /// Every frame of every die, at rest.
    pub fn all_frames(&self) -> Vec<SPTransformStamped> {
        self.dice
            .iter()
            .flat_map(|d| vec![self.pose_frame(d), self.resting_body_frame(d), self.pips_frame(d)])
            .collect()
    }
}

pub fn iso(translation: Vector3<f64>, rotation: UnitQuaternion<f64>) -> Isometry3<f64> {
    Isometry3::from_parts(Translation3::from(translation), rotation)
}

pub fn stamped(child: &str, parent: &str, iso: Isometry3<f64>, metadata: MapOrUnknown, active: bool) -> SPTransformStamped {
    SPTransformStamped {
        active_transform: active,
        enable_transform: true,
        time_stamp: SystemTime::now(),
        parent_frame_id: parent.to_string(),
        child_frame_id: child.to_string(),
        transform: isometry_to_sp_transform(iso),
        metadata,
    }
}

pub fn mesh_metadata(mesh: &str, rgb: [f64; 3]) -> MapOrUnknown {
    MapOrUnknown::Map(vec![
        ("visualize_mesh".to_spvalue(), true.to_spvalue()),
        ("mesh_type".to_spvalue(), 10i64.to_spvalue()),
        ("mesh_file".to_spvalue(), mesh.to_spvalue()),
        ("mesh_scale".to_spvalue(), 0.001.to_spvalue()),
        ("mesh_r".to_spvalue(), rgb[0].to_spvalue()),
        ("mesh_g".to_spvalue(), rgb[1].to_spvalue()),
        ("mesh_b".to_spvalue(), rgb[2].to_spvalue()),
        ("mesh_a".to_spvalue(), 1.0.to_spvalue()),
    ])
}
