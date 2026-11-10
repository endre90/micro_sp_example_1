//! The simulated cell: dice, gripper, camera and operator.
//!
//! Waits for `/tmp/runner_ready.flag` in the launchers - the runner seeds the
//! variables this writes, and seeding them after this has written its first
//! camera reading would put the cell and the model out of step.

use std::sync::Arc;

use dice_cell::cell::{Cell, reset_gripper, run_cell, world::World};
use dice_cell::*;
use micro_sp::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    initialize_env_logger();
    let config = Config::from_env();

    let seed: u64 = match std::env::var("CELL_SEED").ok().and_then(|s| s.trim().parse().ok()) {
        Some(seed) => seed,
        None => std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_millis() as u64,
    };
    log::info!(target: "cell", "{} dice, {} pads, seed {seed}.", config.dice.len(), config.pads);

    let connection_manager = Arc::new(ConnectionManager::new().await);
    let mut con = connection_manager.get_connection().await;

    // The static scene - feeder, arena, tower, pads and the gripper's TCPs.
    // The visualization bridge loads the same directory, but the robot's IK
    // needs these frames whether or not anything is drawing them.
    TransformsManager::load_transforms_from_path(&mut con, &config.scenario_dir).await?;
    log::info!(target: "cell", "Loaded the scene from {}.", config.scenario_dir);

    reset_gripper(&mut con).await;
    let mut cell = Cell::new(World::new(config, seed), &connection_manager).await;
    cell.publish_all().await;
    for die in &cell.world.dice {
        log::info!(target: "cell", "{:>7}: {:?}, showing {}.", die.name, die.loc, die.face());
    }
    log::info!(target: "cell", "Online. Disturb the cell by writing to `disturbance`.");

    run_cell(cell).await;
    Ok(())
}
