//! Automatic transitions: the model looking at the cell again.
//!
//! The planner reasons over the belief variables. When something the model did
//! not do changes the cell - an operator knocks the tower over, takes a die
//! away, a die lands somewhere other than where it was put, the die the robot
//! took out of the feeder turns out to be another colour than the plan hoped -
//! the `cell` binary updates what it reports and `world_changed` is raised. These transitions
//! then copy the camera's picture into the belief, and make sure the plan made
//! for the old picture is not continued:
//!
//! * a plan is running: `replan_pending` holds its next step back, the step
//!   goes stale, and that fails the plan and replans the goal (tasks.rs);
//! * no plan is running, but there was a goal: the goal is replanned straight
//!   away - which is how a finished tower gets rebuilt after it is knocked over.
//!
//! They wait for the transfer skill to be idle, so a die is never re-read while
//! the robot has it in hand.

use micro_sp::*;

use crate::Config;

fn parse(name: &str, guard: &str, actions: &[String], state: &State) -> Transition {
    Transition::parse(
        name,
        guard,
        "true",
        actions.iter().map(|s| s.as_str()).collect(),
        Vec::<&str>::new(),
        state,
    )
}

/// Belief <- camera, for every die, level and pad.
fn copy_measured(config: &Config) -> Vec<String> {
    let mut actions = vec!["var:world_changed <- false".to_string()];
    for d in &config.dice {
        actions.push(format!("var:{d}_seen <- var:{d}_seen_measured"));
        actions.push(format!("var:{d}_in_arena <- var:{d}_in_arena_measured"));
    }
    for k in 1..=config.levels() {
        actions.push(format!("var:level_{k} <- var:level_{k}_measured"));
        actions.push(format!("var:level_{k}_face <- var:level_{k}_face_measured"));
    }
    for j in 1..=config.pads {
        actions.push(format!("var:pad_{j} <- var:pad_{j}_measured"));
    }
    actions
}

pub fn auto_transitions(config: &Config, state: &State) -> Vec<Transition> {
    let sp = &config.sp_id;
    let changed = "var:world_changed == true && var:transfer_busy == false";
    let had_goal = format!("var:{sp}_current_goal_id != UNKNOWN_string");
    let no_goal = format!("var:{sp}_current_goal_id == UNKNOWN_string");
    let executing = format!("var:{sp}_plan_state == executing");
    let not_executing = format!("var:{sp}_plan_state != executing");

    // Only hold the plan back; its next step going stale asks for the replan.
    // Asking here as well would let the goal runner replan at once, which it
    // does while it reads the goal as "initial" - and that abandons a step.
    let mut while_planning = copy_measured(config);
    while_planning.push("var:replan_pending <- true".to_string());

    let mut after_goal = copy_measured(config);
    after_goal.push(format!("var:{sp}_replan_for_same_goal <- true"));
    after_goal.push("var:kpi_run_replans += 1".to_string());

    vec![
        parse("resync_while_executing", &format!("{changed} && {had_goal} && {executing}"), &while_planning, state),
        parse("resync_after_goal", &format!("{changed} && {had_goal} && {not_executing}"), &after_goal, state),
        parse("resync_before_any_goal", &format!("{changed} && {no_goal}"), &copy_measured(config), state),
        // The plan ended while it was being held back - it failed, or its last
        // step was already under way when the world changed and it ran out of
        // steps before one could go stale. Either way it was made for a cell
        // that is gone, so plan the goal again; if the goal happens to hold
        // already, the planner says so and nothing moves. And stop holding
        // back, or the first step of the *next* plan would go stale for nothing.
        parse(
            "replan_after_held_plan",
            &format!("var:replan_pending == true && var:transfer_busy == false && {not_executing} && {had_goal}"),
            &[
                "var:replan_pending <- false".to_string(),
                format!("var:{sp}_replan_for_same_goal <- true"),
                "var:kpi_run_replans += 1".to_string(),
            ],
            state,
        ),
        parse(
            "clear_replan_pending",
            &format!("var:replan_pending == true && var:transfer_busy == false && {not_executing} && {no_goal}"),
            &["var:replan_pending <- false".to_string()],
            state,
        ),
    ]
}
