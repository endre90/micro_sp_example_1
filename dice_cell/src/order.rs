//! Orders: how an operator says what tower they want.
//!
//! An order is one line written into the `order` variable, bottom level first:
//!
//! ```text
//! red:6 green:5 blue:4        red on the bottom showing a six, then green, then blue
//! yellow:1, purple:1          commas, spaces and '>' all separate
//! clear                       no tower at all - everything comes off
//! ```
//!
//! Applying it writes `order_k` / `order_k_face` for every level and
//! `{die}_wanted_face` for every die in one write, and then either posts the
//! tower goal or - if a plan for it is running - holds that plan's next step
//! back, so it goes stale and the goal is replanned against the new order. The goal itself never
//! changes (see model/goal.rs); only what it means does.

use std::sync::Arc;

use micro_sp::running::goal_runner::{GoalPriority, goal_string_to_sp_value};
use micro_sp::*;
use tokio::time::{Duration, interval};

use crate::model::goal::tower_goal;
use crate::model::state::NONE;
use crate::Config;

const T: &str = "order_handler";

/// One level of an order.
#[derive(Debug, Clone, PartialEq)]
pub struct Level {
    pub die: String,
    pub face: i64,
}

/// Parses an order. `Ok(vec![])` is "clear".
pub fn parse_order(text: &str, config: &Config) -> Result<Vec<Level>, String> {
    let text = text.trim();
    if text.eq_ignore_ascii_case("clear") || text.eq_ignore_ascii_case("none") {
        return Ok(vec![]);
    }
    let mut levels: Vec<Level> = vec![];
    for token in text.split(|c: char| c.is_whitespace() || c == ',' || c == '>').filter(|t| !t.is_empty()) {
        let (die, face) = token
            .split_once(':')
            .ok_or_else(|| format!("'{token}' is not colour:face, e.g. red:6"))?;
        let die = die.trim().to_ascii_lowercase();
        if !config.dice.contains(&die) {
            return Err(format!("there is no {die} die - the cell has {}", config.dice.join(", ")));
        }
        if levels.iter().any(|l| l.die == die) {
            return Err(format!("{die} is in the order twice - there is only one {die} die"));
        }
        let face: i64 = face
            .trim()
            .parse()
            .ok()
            .filter(|f| (1..=6).contains(f))
            .ok_or_else(|| format!("'{face}' is not a face - use 1 to 6"))?;
        levels.push(Level { die, face });
    }
    if levels.is_empty() {
        return Err("the order is empty - write e.g. 'red:6 green:5', or 'clear'".to_string());
    }
    if levels.len() > config.levels() {
        return Err(format!("{} levels, but the tower holds at most {}", levels.len(), config.levels()));
    }
    Ok(levels)
}

pub fn describe(levels: &[Level]) -> String {
    if levels.is_empty() {
        return "clear".to_string();
    }
    levels.iter().map(|l| format!("{}:{}", l.die, l.face)).collect::<Vec<_>>().join(" ")
}

/// The variable updates that make `levels` the current order.
pub fn order_state(levels: &[Level], config: &Config) -> State {
    let mut state = State::new();
    let mut set = |name: String, value: SPValue| {
        state.add_mut(SPAssignment::new(SPVariable::new(&name, value.has_type()), value), T);
    };
    for k in 1..=config.levels() {
        let (die, face) = match levels.get(k - 1) {
            Some(l) => (l.die.clone(), l.face),
            None => (NONE.to_string(), 0),
        };
        set(format!("order_{k}"), die.to_spvalue());
        set(format!("order_{k}_face"), face.to_spvalue());
    }
    for die in &config.dice {
        let face = levels.iter().find(|l| &l.die == die).map(|l| l.face).unwrap_or(0);
        set(format!("{die}_wanted_face"), face.to_spvalue());
    }
    state
}

/// Watches `order` and applies it whenever it differs from `order_applied`.
/// Runs inside the runner process, next to micro_sp's own runners.
pub async fn order_handler(config: Config, connection_manager: Arc<ConnectionManager>) {
    let sp = config.sp_id.clone();
    let keys: Vec<String> = vec![
        "order".to_string(),
        "order_applied".to_string(),
        "kpi_run_orders".to_string(),
        format!("{sp}_plan_state"),
        format!("{sp}_incoming_goals"),
        format!("{sp}_scheduled_goals"),
    ];
    let goal = tower_goal(&config);
    let mut con = connection_manager.get_connection().await;
    let mut ticker = interval(Duration::from_millis(100));
    log::info!(target: T, "Online. Write an order into `order`, e.g. 'red:6 green:5 blue:4'.");

    loop {
        ticker.tick().await;
        let Some(state) = StateManager::get_state_for_keys(&mut con, &keys, T).await else {
            continue;
        };
        let order = state.get_string_or_value("order", String::new(), T);
        let applied = state.get_string_or_value("order_applied", String::new(), T);
        if order.trim() == applied.trim() || order.trim().is_empty() {
            continue;
        }

        let mut update = State::new();
        let mut put = |name: &str, value: SPValue| {
            update.add_mut(SPAssignment::new(SPVariable::new(name, value.has_type()), value), T);
        };
        put("order_applied", order.clone().to_spvalue());

        match parse_order(&order, &config) {
            Err(reason) => {
                log::warn!(target: T, "Rejected order '{order}': {reason}.");
                put("order_status", format!("rejected '{order}': {reason}").to_spvalue());
                StateManager::set_state(&mut con, &update).await;
            }
            Ok(levels) => {
                let text = describe(&levels);
                // A plan can be executing while the goal runner reads "initial":
                // a replan for the same goal does not put the goal back to
                // executing. The plan state is the one to trust.
                let executing = state.get_string_or_default_to_unknown(&format!("{sp}_plan_state"), T) == "executing";
                let queued = !state.get_array_or_default_to_empty(&format!("{sp}_incoming_goals"), T).is_empty()
                    || !state.get_array_or_default_to_empty(&format!("{sp}_scheduled_goals"), T).is_empty();

                update.extend_mut(order_state(&levels, &config), true);
                let mut put = |name: &str, value: SPValue| {
                    update.add_mut(SPAssignment::new(SPVariable::new(name, value.has_type()), value), T);
                };
                put("kpi_current_order", text.clone().to_spvalue());
                put("kpi_run_orders", (state.get_int_or_default_to_zero("kpi_run_orders", T) + 1).to_spvalue());

                if executing {
                    // The tower goal is already being worked on. Its meaning just
                    // changed, so hold the old plan's next step back: it goes
                    // stale, and that replans the goal. Not replan_for_same_goal
                    // directly - the goal runner may act on that at once, and
                    // abandon a step whose transfer is still under way.
                    put("replan_pending", true.to_spvalue());
                    put("order_status", format!("applied '{text}' - replanning the running goal").to_spvalue());
                    log::info!(target: T, "New order '{text}' while building - replanning.");
                } else if queued {
                    put("order_status", format!("applied '{text}' - the tower goal is already queued").to_spvalue());
                    log::info!(target: T, "New order '{text}', goal already queued.");
                } else {
                    let posted = vec![goal_string_to_sp_value("", &goal, GoalPriority::Normal)];
                    put(&format!("{sp}_incoming_goals"), posted.to_spvalue());
                    put("order_status", format!("applied '{text}' - building").to_spvalue());
                    log::info!(target: T, "New order '{text}' - goal posted.");
                }
                StateManager::set_state(&mut con, &update).await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orders_parse_and_reject() {
        let config = Config::for_tests(6, 3);
        assert_eq!(
            parse_order("red:6, green:5 > blue:4", &config).unwrap(),
            vec![
                Level { die: "red".into(), face: 6 },
                Level { die: "green".into(), face: 5 },
                Level { die: "blue".into(), face: 4 }
            ]
        );
        assert_eq!(parse_order("clear", &config).unwrap(), vec![]);
        assert!(parse_order("red:7", &config).is_err());
        assert!(parse_order("red:1 red:2", &config).is_err());
        assert!(parse_order("pink:1", &config).is_err());
        assert!(parse_order("red", &config).is_err());
        assert!(parse_order("orange:1", &Config::for_tests(3, 2)).is_err());
    }
}
