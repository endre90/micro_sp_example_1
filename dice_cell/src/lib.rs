//! A dice-stacking cell for micro_sp.
//!
//! A UR5 builds a tower of coloured dice. Each die has to show a given face. An
//! *order* says which colour goes on which level and which face it shows. The
//! dice come out of a blind feeder - nobody knows which die is in which slot -
//! and are thrown into the arena, where a camera reads their colour and face.
//! A die going onto the tower is turned at the flip station on the way, if it
//! needs to be; one that came out before its turn waits on a buffer pad. The
//! planner works out the rest from whatever the cell looks like right now -
//! dice not seen yet, dice lying in the arena, a half-built tower that is wrong
//! at level two.
//!
//! Two binaries share this crate:
//!
//! * `runner` - the control system. Builds the model, seeds the state, runs
//!   micro_sp, and turns an order into a goal.
//! * `cell` - everything physical that is not the robot: the dice, the gripper,
//!   the feeder, the camera over the arena, and the disturbances an operator
//!   can inject (knock the tower over, take a die away, ...).
//!
//! The robot itself is `simple_robot_simulator`, a separate process.

pub mod layout;
pub mod model;
pub mod order;
pub mod cell;

pub use layout::*;

use micro_sp::*;

/// Reads `name`, falling back to `default` and saying so. Both binaries read
/// the same variables, and a disagreement between them is a stack that comes
/// up and never talks to itself - so every fallback is logged.
pub fn env_or(name: &str, default: &str) -> String {
    match std::env::var(name) {
        Ok(value) if !value.trim().is_empty() => value.trim().to_string(),
        _ => {
            log::info!(target: "env", "{name} not set, using '{default}'.");
            default.to_string()
        }
    }
}

fn env_usize(name: &str, default: usize, min: usize, max: usize) -> usize {
    let raw = env_or(name, &default.to_string());
    match raw.parse::<usize>() {
        Ok(v) if (min..=max).contains(&v) => v,
        _ => {
            log::warn!(target: "env", "{name}='{raw}' is not a number in {min}..={max}, using {default}.");
            default
        }
    }
}

fn env_f64(name: &str, default: f64) -> f64 {
    let raw = env_or(name, &default.to_string());
    raw.parse::<f64>().unwrap_or_else(|_| {
        log::warn!(target: "env", "{name}='{raw}' is not a number, using {default}.");
        default
    })
}

/// The settings both binaries agree on.
#[derive(Debug, Clone)]
pub struct Config {
    pub sp_id: String,
    pub robot_id: String,
    /// The dice in the cell, by colour. Also their ids in the state.
    pub dice: Vec<String>,
    pub pads: usize,
    /// How fast the arm moves; simple_robot_simulator's `velocity`.
    pub robot_speed: f64,
    /// How long a planned step may wait for its precondition before the plan
    /// is declared stale and the goal is replanned.
    pub stale_step_ms: i64,
    pub scenario_dir: String,
}

impl Config {
    pub fn from_env() -> Config {
        let n_dice = env_usize("N_DICE", 6, 2, MAX_DICE);
        Config {
            sp_id: env_or("SP_INSTANCE_ID", "sp1"),
            robot_id: env_or("ROBOT_ID", "r1"),
            dice: DIE_COLOURS.iter().take(n_dice).map(|c| c.name.to_string()).collect(),
            pads: env_usize("N_PADS", 3, 1, MAX_PADS),
            robot_speed: env_f64("ROBOT_SPEED", 30.0),
            stale_step_ms: env_usize("STALE_STEP_MS", 1000, 100, 60_000) as i64,
            scenario_dir: env_or("SCENARIO_DIR", "shared_folder/scenario"),
        }
    }

    /// The tower can be as tall as there are dice.
    pub fn levels(&self) -> usize {
        self.dice.len()
    }

    /// A config for tests and tools that do not read the environment.
    pub fn for_tests(n_dice: usize, pads: usize) -> Config {
        Config {
            sp_id: "sp1".to_string(),
            robot_id: "r1".to_string(),
            dice: DIE_COLOURS.iter().take(n_dice).map(|c| c.name.to_string()).collect(),
            pads,
            robot_speed: 30.0,
            stale_step_ms: 1000,
            scenario_dir: "shared_folder/scenario".to_string(),
        }
    }
}

/// Puts one `{prefix}_request_*` channel back to rest, so a trigger left true
/// by a killed process does not fire the moment the runners start.
pub async fn reset_request_channel(con: &mut SPConnection, prefix: &str) {
    StateManager::set_sp_value(con, &format!("{prefix}_request_state"), &"initial".to_spvalue()).await;
    StateManager::set_sp_value(con, &format!("{prefix}_request_trigger"), &false.to_spvalue()).await;
}
