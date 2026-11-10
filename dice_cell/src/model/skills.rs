//! The skills: two SOPs, one per way the robot handles a die.
//!
//! * **Transfer** - move one die from where it is to where it should go, and set
//!   it down or throw it.
//! * **Build** - take one die to the tower, turning it at the flip station on
//!   the way if it does not show the face the order wants yet: tip it onto the
//!   station on its side (once or twice), then pick it up from there and stack
//!   it. A die that already shows the right face goes straight to the tower.
//!
//! This is the half of the cell where the *route* is the requirement, so these
//! are fixed sequences rather than something planned. Both follow the same rules,
//! which are what keep the arm clear of the table, the dice and the tower:
//!
//! * travel only at SAFE_Z, through the `*_transit` point above a place, with a
//!   joint move (`move_j`);
//! * go down to and up from a place only in a straight vertical line
//!   (`move_l`), fast to 10 cm above it, then slowly the rest of the way.
//!
//! The planned operations in tasks.rs never see these steps - they hand the SOPs
//! their parameters (which die, which frame to pick it at, which frame to put it
//! on, place or throw) through the `transfer_*` variables and wait for them to
//! finish. Planning at the task level, SOPs at the skill level, and the planning
//! domain stays small.

use micro_sp::*;

use crate::{
    BUILD_PICK_FRAME, Config, FLIP_GRIP_FRAME, FLIP_STATION_FRAME, FLIP_STATION_TILT_ABOVE_FRAME,
    FLIP_STATION_TILT_FRAME, TCP_ABOVE_FRAME, TCP_FRAME, transit_frame,
};

pub const TRANSFER_SOP: &str = "sop_transfer";
pub const BUILD_SOP: &str = "sop_build";

/// The last few centimetres onto or off a die go at this share of the speed.
const SLOW: f64 = 0.45;

/// A frame name given either literally or as the variable that holds it.
#[derive(Clone, Copy)]
pub enum Frame<'a> {
    Named(&'a str),
    FromVar(&'a str),
}

impl Frame<'_> {
    fn rhs(&self) -> String {
        match self {
            Frame::Named(name) => name.to_string(),
            Frame::FromVar(var) => format!("var:{var}"),
        }
    }
}

fn parse(name: &str, guard: &str, runner_guard: &str, actions: &[String], runner_actions: &[String], state: &State) -> Transition {
    Transition::parse(
        name,
        guard,
        runner_guard,
        actions.iter().map(|s| s.as_str()).collect(),
        runner_actions.iter().map(|s| s.as_str()).collect(),
        state,
    )
}

/// How a motion moves: `Joint` (`move_j`) for travel between transit points,
/// `Line` (`move_l`) for going straight down to a place and back up, `Slow`
/// for the last stretch of that.
#[derive(Clone, Copy)]
pub enum Motion {
    Joint,
    Line,
    Slow,
}

/// One motion request to the robot, waiting for it to succeed. A failed motion
/// is retried once, then the SOP fails - and the planned operation driving it
/// with it.
pub fn robot_move(name: &str, config: &Config, motion: Motion, goal: Frame, tcp: &str, state: &State) -> Operation {
    let r = &config.robot_id;
    let (command, speed) = match motion {
        Motion::Joint => ("move_j", config.robot_speed),
        Motion::Line => ("move_l", config.robot_speed),
        Motion::Slow => ("move_l", config.robot_speed * SLOW),
    };
    robot_request(
        name,
        config,
        vec![
            format!("var:{r}_command_type <- {command}"),
            format!("var:{r}_use_joint_positions <- false"),
            format!("var:{r}_baseframe_id <- base_link"),
            format!("var:{r}_faceplate_id <- tool0"),
            format!("var:{r}_goal_feature_id <- {}", goal.rhs()),
            format!("var:{r}_tcp_id <- {tcp}"),
            format!("var:{r}_velocity <- {speed:.3}"),
        ],
        state,
    )
}

/// One request on the robot's channel: write the parameters, raise the
/// trigger, wait for `succeeded`.
///
/// A motion that does not finish in a minute is cancelled and the channel put
/// back to rest, so the next request finds it usable; the request state is
/// re-armed to `initial` on every start for the same reason, since a cancelled
/// motion ends as `succeeded`.
fn robot_request(name: &str, config: &Config, parameters: Vec<String>, state: &State) -> Operation {
    let r = &config.robot_id;
    let mut start = parameters;
    start.push(format!("var:{r}_request_state <- initial"));
    start.push(format!("var:{r}_request_trigger <- true"));
    Operation::new(
        name,
        Some(60_000),
        Some(30_000),
        Some(1),
        None,
        false,
        vec![parse(
            &format!("start_{name}"),
            &format!("var:{r}_request_state != executing && var:{r}_request_trigger == false"),
            "true",
            &start,
            &[],
            state,
        )],
        vec![parse(
            &format!("complete_{name}"),
            "true",
            &format!("var:{r}_request_state == succeeded"),
            &[
                format!("var:{r}_request_trigger <- false"),
                format!("var:{r}_request_state <- initial"),
            ],
            &[],
            state,
        )],
        vec![parse(
            &format!("fail_{name}"),
            "true",
            &format!("var:{r}_request_state == failed"),
            &[
                format!("var:{r}_request_trigger <- false"),
                format!("var:{r}_request_state <- initial"),
            ],
            &[],
            state,
        )],
        vec![parse(
            &format!("timeout_{name}"),
            "true",
            "true",
            &[
                format!("var:{r}_request_cancel <- true"),
                format!("var:{r}_request_trigger <- false"),
            ],
            &[],
            state,
        )],
        vec![],
        vec![],
    )
}

/// One gripper request, served by the `cell` binary:
///
/// * `grip` closes on `transfer_die` - and fails when the die is not between
///   the fingers;
/// * `release` lets go of it as `transfer_release` says: `place` sets it down
///   on `target`, `throw` lets it tumble into the arena;
/// * `place` is `release` with `place`, whatever `transfer_release` says;
/// * `look` finds `transfer_die`, publishes where it lies as `build_pick`, and
///   says whether it still has to be turned (`gripper_flip_needed`); a `place`
///   on the flip station says the same;
/// * `plan_flip` works out how to grip `transfer_die` for its next flip and
///   publishes it as `flip_grip`.
pub fn gripper_command(name: &str, command: &str, target: Frame, state: &State) -> Operation {
    gripper_command_when(name, command, target, "true", state)
}

/// [`gripper_command`], starting only once `guard` holds as well.
pub fn gripper_command_when(name: &str, command: &str, target: Frame, guard: &str, state: &State) -> Operation {
    let command_rhs = match command {
        "release" => "var:transfer_release".to_string(),
        other => other.to_string(),
    };
    Operation::new(
        name,
        Some(10_000),
        Some(30_000),
        None,
        None,
        false,
        vec![parse(
            &format!("start_{name}"),
            &format!("var:gripper_request_state == initial && var:gripper_request_trigger == false && {guard}"),
            "true",
            &[
                format!("var:gripper_command <- {command_rhs}"),
                "var:gripper_die <- var:transfer_die".to_string(),
                format!("var:gripper_target <- {}", target.rhs()),
                "var:gripper_request_trigger <- true".to_string(),
            ],
            &[],
            state,
        )],
        vec![parse(
            &format!("complete_{name}"),
            "true",
            "var:gripper_request_state == succeeded",
            &[
                "var:gripper_request_trigger <- false".to_string(),
                "var:gripper_request_state <- initial".to_string(),
            ],
            &[],
            state,
        )],
        vec![parse(
            &format!("fail_{name}"),
            "true",
            "var:gripper_request_state == failed",
            &[
                "var:gripper_request_trigger <- false".to_string(),
                "var:gripper_request_state <- initial".to_string(),
            ],
            &[],
            state,
        )],
        vec![],
        vec![],
        vec![],
    )
}

fn step(op: Operation) -> SOP {
    SOP::Operation(Box::new(op))
}

/// Pick `transfer_die` at `transfer_pick`, take it to `transfer_place` and let
/// go of it with `transfer_release`, travelling through `transfer_pick_transit`
/// and `transfer_place_transit`.
pub fn transfer_sop(config: &Config, state: &State) -> SOPStruct {
    use Motion::*;
    let pick = Frame::FromVar("transfer_pick");
    let pick_transit = Frame::FromVar("transfer_pick_transit");
    let place = Frame::FromVar("transfer_place");
    let place_transit = Frame::FromVar("transfer_place_transit");
    let mv = |name: &str, motion, goal, tcp| step(robot_move(name, config, motion, goal, tcp, state));
    SOPStruct {
        id: TRANSFER_SOP.to_string(),
        sop: SOP::Sequence(vec![
            mv("transfer_travel_to_pick", Joint, pick_transit, TCP_FRAME),
            mv("transfer_approach_pick", Line, pick, TCP_ABOVE_FRAME),
            mv("transfer_descend_pick", Slow, pick, TCP_FRAME),
            step(gripper_command("transfer_grip", "grip", place, state)),
            mv("transfer_lift", Line, pick_transit, TCP_FRAME),
            mv("transfer_travel_to_place", Joint, place_transit, TCP_FRAME),
            mv("transfer_approach_place", Line, place, TCP_ABOVE_FRAME),
            mv("transfer_descend_place", Slow, place, TCP_FRAME),
            step(gripper_command("transfer_release", "release", place, state)),
            mv("transfer_retract", Line, place_transit, TCP_FRAME),
        ]),
    }
}

/// An SOP step that does nothing but wait for `guard` - the way into a branch
/// of an `Alternative` that has nothing to do.
fn gate(name: &str, guard: &str, state: &State) -> Operation {
    Operation::new(
        name,
        Some(10_000),
        Some(30_000),
        None,
        None,
        false,
        vec![parse(&format!("start_{name}"), guard, "true", &[], &[], state)],
        vec![parse(&format!("complete_{name}"), "true", "true", &[], &[], state)],
        vec![],
        vec![],
        vec![],
        vec![],
    )
}

/// One quarter turn at the flip station, from wherever the die lies - the arena,
/// a pad, or the station itself after an earlier flip:
///
/// 1. the cell works out how to grip the die so that tipping it brings the right
///    face up, and publishes that grip as `flip_grip`;
/// 2. pick it up there;
/// 3. travel to the flip station, tip the tool over in the air - the die turns a
///    quarter in the fingers - and lower it onto the post on its side;
/// 4. let go and rise. The cell says whether that was enough.
fn flip_once(n: usize, config: &Config, state: &State) -> SOP {
    use Motion::*;
    let named = Frame::Named;
    let grip = FLIP_GRIP_FRAME;
    let grip_transit = transit_frame(FLIP_GRIP_FRAME);
    let station = FLIP_STATION_FRAME;
    let station_transit = transit_frame(FLIP_STATION_FRAME);
    let mv = |name: &str, motion, goal, tcp| step(robot_move(&format!("build_flip{n}_{name}"), config, motion, goal, tcp, state));
    SOP::Sequence(vec![
        step(gripper_command_when(&format!("build_flip{n}_plan"), "plan_flip", named(station), "var:gripper_flip_needed == true", state)),
        mv("travel_to_die", Joint, named(&grip_transit), TCP_FRAME),
        mv("approach_die", Line, named(grip), TCP_ABOVE_FRAME),
        mv("descend_to_die", Slow, named(grip), TCP_FRAME),
        step(gripper_command(&format!("build_flip{n}_grip"), "grip", named(station), state)),
        mv("lift_die", Line, named(&grip_transit), TCP_FRAME),
        mv("travel_to_station", Joint, named(&station_transit), TCP_FRAME),
        mv("tip_over", Line, named(FLIP_STATION_TILT_ABOVE_FRAME), TCP_FRAME),
        mv("lower_onto_station", Slow, named(FLIP_STATION_TILT_FRAME), TCP_FRAME),
        step(gripper_command(&format!("build_flip{n}_set_down"), "place", named(station), state)),
        mv("rise_from_station", Line, named(FLIP_STATION_TILT_ABOVE_FRAME), TCP_FRAME),
    ])
}

/// Take `transfer_die` from wherever it lies to `transfer_place`, turning it on
/// the way until it shows `{die}_wanted_face`:
///
/// 1. the cell looks at the die: where it is (`build_pick`) and whether it
///    already shows the wanted face (`gripper_flip_needed`);
/// 2. up to two quarter turns at the flip station - a face on a side is one
///    away, the one underneath two - each one only if it is still needed;
/// 3. pick the die up from where it now lies - the arena, a pad, or the station -
///    and set it down at `transfer_place`.
///
/// So a turned die goes straight from the flip station to the tower, and one
/// that already shows the right face goes straight there from where it lay.
pub fn build_sop(config: &Config, state: &State) -> SOPStruct {
    use Motion::*;
    let pick = Frame::Named(BUILD_PICK_FRAME);
    let pick_transit = transit_frame(BUILD_PICK_FRAME);
    let pick_transit = Frame::Named(&pick_transit);
    let place = Frame::FromVar("transfer_place");
    let place_transit = Frame::FromVar("transfer_place_transit");
    let mv = |name: &str, motion, goal, tcp| step(robot_move(name, config, motion, goal, tcp, state));
    let flip_or_not = |n: usize| {
        SOP::Alternative(vec![
            flip_once(n, config, state),
            step(gate(&format!("build_flip{n}_not_needed"), "var:gripper_flip_needed == false", state)),
        ])
    };
    SOPStruct {
        id: BUILD_SOP.to_string(),
        sop: SOP::Sequence(vec![
            step(gripper_command("build_look", "look", place, state)),
            flip_or_not(1),
            flip_or_not(2),
            mv("build_travel_to_pick", Joint, pick_transit, TCP_FRAME),
            mv("build_approach_pick", Line, pick, TCP_ABOVE_FRAME),
            mv("build_descend_pick", Slow, pick, TCP_FRAME),
            step(gripper_command("build_grip", "grip", place, state)),
            mv("build_lift", Line, pick_transit, TCP_FRAME),
            mv("build_travel_to_place", Joint, place_transit, TCP_FRAME),
            mv("build_approach_place", Line, place, TCP_ABOVE_FRAME),
            mv("build_descend_place", Slow, place, TCP_FRAME),
            step(gripper_command("build_release", "place", place, state)),
            mv("build_retract", Line, place_transit, TCP_FRAME),
        ]),
    }
}
