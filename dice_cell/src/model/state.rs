//! Every variable the model reads or writes, with the value it starts at.
//!
//! `Transition::parse` resolves `var:` names at parse time, so this has to be
//! built - and merged into the state the operations are parsed against - before
//! any operation is. There are three kinds:
//!
//! * **Belief** - what the control system thinks the cell looks like. The
//!   planner searches over these, and planned operations write them.
//! * **Measured** - what the cell's camera says. Only the `cell` binary writes
//!   these; the model copies them into the belief when the world changes
//!   behind its back (see `reactions.rs`).
//! * **Interfaces** - the request channels of the robot and the gripper, and
//!   the order and disturbance inputs an operator writes from the GUI.

use micro_sp::*;
use ordered_float::OrderedFloat;

use crate::Config;

pub const NONE: &str = "none";
const T: &str = "dice_state";

fn add(state: &mut State, var: SPVariable, value: SPValue) {
    state.add_mut(SPAssignment::new(var, value), T);
}

fn s(state: &mut State, name: &str, value: &str) {
    add(state, SPVariable::new(name, SPValueType::String), value.to_spvalue());
}
fn s_unknown(state: &mut State, name: &str) {
    add(state, SPVariable::new(name, SPValueType::String), SPValue::String(StringOrUnknown::UNKNOWN));
}
fn b(state: &mut State, name: &str, value: bool) {
    add(state, SPVariable::new(name, SPValueType::Bool), value.to_spvalue());
}
fn i(state: &mut State, name: &str, value: i64) {
    add(state, SPVariable::new(name, SPValueType::Int64), value.to_spvalue());
}
fn f(state: &mut State, name: &str, value: f64) {
    add(state, SPVariable::new(name, SPValueType::Float64), SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(value))));
}
fn a(state: &mut State, name: &str) {
    add(state, SPVariable::new(name, SPValueType::Array), SPValue::Array(ArrayOrUnknown::Array(vec![])));
}

/// What the control system believes, the order it is working towards, and the
/// handshakes between the planned operations and the SOPs.
///
/// Starts from "no die has been seen": the cell's first camera reading replaces
/// that within a tick of the cell coming up.
pub fn belief_and_order_variables(config: &Config) -> State {
    let mut state = State::new();
    for die in &config.dice {
        // Seen: the camera over the arena has read this die's colour, so the
        // model knows where it is - in the arena, on a pad, on the tower. A die
        // that has not been seen is in the feeder, but nobody knows which slot:
        // the feeder has no camera, and its dice all look the same from above.
        b(&mut state, &format!("{die}_seen"), false);
        // Lying in the arena, where the camera can see it.
        b(&mut state, &format!("{die}_in_arena"), false);
        // What the order wants this die to show, 0 if it is not in the order.
        // Written only by the order handler, together with order_k / order_k_face.
        i(&mut state, &format!("{die}_wanted_face"), 0);
    }
    for k in 1..=config.levels() {
        s(&mut state, &format!("level_{k}"), NONE);
        i(&mut state, &format!("level_{k}_face"), 0);
        s(&mut state, &format!("order_{k}"), NONE);
        i(&mut state, &format!("order_{k}_face"), 0);
    }
    // The buffer pads. A die on one keeps how it lies, and the build SOP turns
    // it on its way to the tower, so the planner only needs to know which die.
    for j in 1..=config.pads {
        s(&mut state, &format!("pad_{j}"), NONE);
    }

    // The parameters a planned operation hands the SOPs.
    s(&mut state, "transfer_die", NONE);
    s(&mut state, "transfer_pick", NONE);
    s(&mut state, "transfer_pick_transit", NONE);
    s(&mut state, "transfer_place", NONE);
    s(&mut state, "transfer_place_transit", NONE);
    s(&mut state, "transfer_release", "place");
    s(&mut state, "transfer_task", NONE);
    b(&mut state, "transfer_busy", false);

    // The world moved behind the plan's back - set by the cell, consumed by
    // the resync transition, which sets replan_pending in turn.
    b(&mut state, "world_changed", true);
    b(&mut state, "replan_pending", false);

    // The order, as the operator writes it, e.g. "red:6 blue:3 green:1"
    // (bottom first). See order.rs.
    s(&mut state, "order", "");
    s(&mut state, "order_applied", "");
    s(&mut state, "order_status", "no order yet - write one into `order`, e.g. 'red:6 green:5 blue:4'");
    s(&mut state, "tower_goal", &crate::model::goal::tower_goal(config));

    // KPIs, for the GUI's Production tab (it renders anything named kpi_<group>_<name>).
    i(&mut state, "kpi_run_throws", 0);
    i(&mut state, "kpi_run_other_colour", 0);
    i(&mut state, "kpi_run_flips", 0);
    i(&mut state, "kpi_run_transfers", 0);
    i(&mut state, "kpi_run_replans", 0);
    i(&mut state, "kpi_run_disturbances", 0);
    i(&mut state, "kpi_run_orders", 0);
    s(&mut state, "kpi_current_order", "-");
    s(&mut state, "kpi_current_task", "-");
    i(&mut state, "kpi_model_planned_operations", 0);
    i(&mut state, "kpi_model_operation_templates", 0);
    state
}

/// What the cell reports. Seeded by the runner so the model can be parsed
/// against it, but merged *without* overwriting, so a runner restart does not
/// clobber what a running cell has already published.
///
/// The camera looks at the arena only: colour and face of every die lying
/// there. A die it has seen once stays seen - the cell tracks it to the pads
/// and the tower - until it goes back into the feeder.
pub fn measured_variables(config: &Config) -> State {
    let mut state = State::new();
    for die in &config.dice {
        b(&mut state, &format!("{die}_seen_measured"), false);
        b(&mut state, &format!("{die}_in_arena_measured"), false);
        // 0 unless the die is in the arena, where the camera can read it.
        i(&mut state, &format!("{die}_face_measured"), 0);
    }
    i(&mut state, "feeder_count_measured", 0);
    for k in 1..=config.levels() {
        s(&mut state, &format!("level_{k}_measured"), NONE);
        i(&mut state, &format!("level_{k}_face_measured"), 0);
    }
    for j in 1..=config.pads {
        s(&mut state, &format!("pad_{j}_measured"), NONE);
    }
    // Raised by the cell once it has published the dice.
    b(&mut state, "cell_online", false);
    s(&mut state, "disturbance", "");
    s(&mut state, "disturbance_status", "write a disturbance here: knock_tower, steal_top, steal <colour>, bump <colour>, reset");
    s(&mut state, "gripper_holding", NONE);
    state
}

/// The gripper's request channel, served by the `cell` binary.
pub fn gripper_variables() -> State {
    let mut state = State::new();
    b(&mut state, "gripper_request_trigger", false);
    s(&mut state, "gripper_request_state", "initial");
    s(&mut state, "gripper_command", NONE);
    s(&mut state, "gripper_die", NONE);
    s(&mut state, "gripper_target", NONE);
    // Set by `look` and by a `place` on the flip station: the die does not show
    // its wanted face yet. Steers the build SOP's flips.
    b(&mut state, "gripper_flip_needed", false);
    s_unknown(&mut state, "gripper_result");
    state
}

/// The robot's request channel - simple_robot_simulator's (and ur_redis_driver's)
/// keys, with the values the transfer SOP leaves alone at rest. The simulator
/// seeds these too; they are declared here so the model parses on a Redis the
/// simulator has not reached yet.
pub fn robot_variables(robot_id: &str) -> State {
    let r = robot_id;
    let mut state = State::new();
    b(&mut state, &format!("{r}_request_trigger"), false);
    s(&mut state, &format!("{r}_request_state"), "initial");
    b(&mut state, &format!("{r}_request_cancel"), false);
    s_unknown(&mut state, &format!("{r}_request_result"));
    s_unknown(&mut state, &format!("{r}_command_type"));
    f(&mut state, &format!("{r}_velocity"), 1.0);
    f(&mut state, &format!("{r}_acceleration"), 1.0);
    b(&mut state, &format!("{r}_use_joint_positions"), false);
    a(&mut state, &format!("{r}_joint_positions"));
    s(&mut state, &format!("{r}_baseframe_id"), "base_link");
    s(&mut state, &format!("{r}_faceplate_id"), "tool0");
    s_unknown(&mut state, &format!("{r}_goal_feature_id"));
    s_unknown(&mut state, &format!("{r}_tcp_id"));
    state
}
