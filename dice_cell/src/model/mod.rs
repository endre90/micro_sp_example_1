//! The model: everything the cell can do, and nothing about in which order.
//!
//! | Part | Kind | Where |
//! |---|---|---|
//! | Transfer one die (travel, descend, grip, lift, travel, descend, release) | SOP | skills.rs |
//! | Build: turn one die at the flip station as needed, then stack it | SOP | skills.rs |
//! | Fetch from the feeder / stack / park / unstack / throw off | planned operations | tasks.rs |
//! | Re-read the cell when it changed behind the plan's back | automatic transitions | reactions.rs |
//! | "The tower is the order" | goal | goal.rs |

pub mod goal;
pub mod reactions;
pub mod skills;
pub mod state;
pub mod tasks;

use micro_sp::*;

use crate::Config;

/// Builds the model against `state` - which must already hold micro_sp's runner
/// variables - and returns it together with the state extended by everything
/// the model declares. Seed *that* state, not the one passed in.
pub fn build_model(config: &Config, state: &State) -> (Model, State) {
    let mut state = state.extend(state::belief_and_order_variables(config), true);
    // Owned by other processes: keep what they have already written.
    state.extend_mut(state::measured_variables(config), false);
    state.extend_mut(state::robot_variables(&config.robot_id), false);
    state.extend_mut(state::gripper_variables(), true);

    let sops = vec![skills::transfer_sop(config, &state), skills::build_sop(config, &state)];
    state.extend_mut(generate_multi_sop_state_variables(&sops, "dice_model"), true);

    let operations = tasks::planned_operations(config, &state);
    let auto_transitions = reactions::auto_transitions(config, &state);

    state.update_mut("kpi_model_planned_operations", (operations.len() as i64).to_spvalue());
    state.update_mut("kpi_model_operation_templates", tasks::TEMPLATE_COUNT.to_spvalue());

    let model = Model::new(&config.sp_id, auto_transitions, vec![], vec![], sops, operations);
    (model, state)
}

/// The model's state for planning tests and tools: runner variables plus
/// everything [`build_model`] declares, with no Redis involved.
pub fn offline_state(config: &Config) -> (Model, State) {
    let runner = generate_runner_state_variables(&config.sp_id, 1, "dice_model");
    let (model, state) = build_model(config, &runner);
    let ops = generate_operation_state_variables(&model, false, "dice_model");
    (model, state.extend(ops, true))
}
