//! The one goal this cell ever plans for.
//!
//! "The tower is the order": every level holds the colour the order puts there,
//! showing the face the order asks for, and the levels the order leaves empty
//! are empty. It is written against the order variables rather than against
//! literal colours, so it never has to change - a new order changes what the
//! goal *means*, not the goal itself. That is what lets an order be changed
//! halfway through a build: the running goal is simply replanned.

use crate::Config;

pub fn tower_goal(config: &Config) -> String {
    (1..=config.levels())
        .map(|k| format!("var:level_{k} == var:order_{k} && var:level_{k}_face == var:order_{k}_face"))
        .collect::<Vec<_>>()
        .join(" && ")
}
