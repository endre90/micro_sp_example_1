//! The control system.
//!
//! Builds the model, pushes its state into Redis, starts micro_sp's runners and
//! the order handler, and then stays alive. The last thing it does before
//! idling is write `/tmp/runner_ready.flag`, which is what the launchers hold
//! the `cell` back on: "the state this process owns is in Redis".

use std::sync::Arc;

use dice_cell::model::build_model;
use dice_cell::order::order_handler;
use dice_cell::*;
use micro_sp::*;
use tokio::time::{Duration, interval};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    initialize_env_logger();
    let config = Config::from_env();
    let sp_id = config.sp_id.clone();
    let target = format!("{sp_id}_dice_runner");

    let connection_manager = Arc::new(ConnectionManager::new().await);
    let mut con = connection_manager.get_connection().await;

    // Start from what is already in Redis - the robot simulator's interface,
    // and the camera's last reading if the cell is already up - so that only
    // what this process owns is reset.
    let state = StateManager::get_full_state(&mut con).await.unwrap_or_else(State::new);
    let number_of_timers = 1;
    let state = state.extend(generate_runner_state_variables(&sp_id, number_of_timers, "runner_vars"), true);
    let (model, state) = build_model(&config, &state);
    let state = state.extend(generate_operation_state_variables(&model, false, "op_vars"), true);

    log::info!(
        target: &target,
        "{} dice, {} pads, {} planned operations from {} templates, {} SOPs, {} automatic transitions.",
        config.dice.len(),
        config.pads,
        model.operations.len(),
        dice_cell::model::tasks::TEMPLATE_COUNT,
        model.sops.len(),
        model.auto_transitions.len(),
    );

    // micro_sp's set_state logs a failed MSET instead of returning it, so read
    // one of the seeded variables back and retry until it is there. Without
    // this a Redis that drops the connection at the wrong moment (e.g. it is
    // being recreated) leaves every model variable missing while the runner
    // still reports itself ready.
    let probe = format!("{sp_id}_plan");
    let mut attempt = 1;
    loop {
        StateManager::set_state(&mut con, &state).await;
        if StateManager::get_sp_value(&mut con, &probe).await.is_some() {
            break;
        }
        log::warn!(target: &target, "Seeding the model state failed (attempt {attempt}); retrying.");
        attempt += 1;
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    reset_request_channel(&mut con, &config.robot_id).await;
    reset_request_channel(&mut con, "gripper").await;

    tokio::time::sleep(Duration::from_millis(500)).await;

    let runner_model = model.clone();
    let runner_sp_id = sp_id.clone();
    let runner_connection = connection_manager.clone();
    tokio::task::spawn(async move {
        main_runner(&runner_sp_id, runner_model, number_of_timers, &runner_connection).await
    });

    tokio::task::spawn(order_handler(config.clone(), connection_manager.clone()));

    std::fs::File::create("/tmp/runner_ready.flag")?;
    log::info!(target: &target, "Ready. Write an order into `order`, e.g. 'red:6 green:5 blue:4'.");

    // A demo can start itself: INITIAL_ORDER is written into `order` as if an
    // operator had typed it - but only once the cell is up and has published
    // the dice. Planned any earlier, the first transfer would go looking for a
    // frame that does not exist yet.
    if let Ok(order) = std::env::var("INITIAL_ORDER") {
        let order = order.trim().to_string();
        if !order.is_empty() {
            log::info!(target: &target, "Holding the initial order '{order}' until the cell is up.");
            loop {
                if let Some(SPValue::Bool(BoolOrUnknown::Bool(true))) = StateManager::get_sp_value(&mut con, "cell_online").await {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(250)).await;
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
            log::info!(target: &target, "Placing the initial order '{order}'.");
            StateManager::set_sp_value(&mut con, "order", &order.to_spvalue()).await;
        }
    }

    let mut idle = interval(Duration::from_secs(3600));
    loop {
        idle.tick().await;
    }
}
