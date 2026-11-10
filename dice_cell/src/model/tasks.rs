//! The planned operations: what the cell *can* do, as the planner sees it.
//!
//! Seven templates, instantiated per die, level and pad:
//!
//! | Template | From | To | SOP |
//! |---|---|---|---|
//! | `fetch_{die}` | feeder | arena | transfer (throw) |
//! | `stack_{die}_on_level_{k}` | arena | tower level k | build |
//! | `park_{die}_on_pad_{j}` | arena | pad j | transfer |
//! | `stack_pad_{j}_on_level_{k}` | pad j | tower level k | build |
//! | `unstack_level_{k}_to_pad_{j}` | top of the tower | pad j | transfer |
//! | `throw_{die}_from_level_{k}` | top of the tower | arena | transfer (throw) |
//! | `throw_{die}_from_pad_{j}` | pad j | arena | transfer (throw) |
//!
//! None of them names a sequence. Which die to go for next, what to take off a
//! wrong tower and where to park it, what to do with a die that came out of the
//! feeder before its turn - all of that is the planner's.
//!
//! **The feeder is blind.** Its dice all look the same from above, and the only
//! camera looks at the arena. So the model does not know which die is in which
//! slot - only which dice it has not seen yet. `fetch_{die}` takes the next die
//! out of the feeder and throws it into the arena, and the planner assumes it is
//! `{die}`: the plan is a hopeful one. When the camera says it is another colour,
//! the operation fails, the model takes in what the camera saw, and the goal is
//! replanned with one more die known - typically "park it on a pad and fetch
//! again". So a fetch is only allowed while no die the order wants is lying in
//! the arena (unless every pad is taken): the pads are where the dice that came
//! out too early wait for their turn. A die the order does not want at all is
//! left in the arena.
//!
//! **Faces are the build SOP's business.** A die going onto the tower is turned
//! at the flip station on the way until it shows the face the order wants - none,
//! one or two quarter turns, as the die lies - and goes from the station straight
//! onto the tower. So the planner never reasons about faces of loose dice at
//! all: it only knows that a die stacked on level k shows `order_k_face`.

use micro_sp::*;

use super::skills::{BUILD_SOP, TRANSFER_SOP};
use crate::{ARENA_DROP_FRAME, Config, FEEDER_NEXT_FRAME, UNKNOWN_DIE, die_frame, level_frame, pad_frame, transit_frame};

/// A planned step never times out while its SOP is running: every step of the
/// SOP has its own, shorter timeout, so the SOP always ends - and only then may
/// the step end. A step that gave up first would leave the SOP running with
/// nobody to collect it, and the skills locked.
const PLANNED_STEP_TIMEOUT_MS: i64 = 24 * 3_600_000;

/// The seven templates, for the GUI's Production tab.
pub const TEMPLATE_COUNT: i64 = 7;

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

/// What a planned operation hands an SOP.
enum Move {
    /// The transfer SOP: pick at `pick`, take it to `place`, let go with `release`.
    Transfer { die: String, pick: String, place: String, release: &'static str, label: String },
    /// The build SOP: find the die, turn it as needed, set it down at `place`.
    Build { die: String, place: String, label: String },
}

/// The runner guard every planned operation starts on: the skills are free, and
/// nothing has told us the plan is stale.
const READY: &str = "var:transfer_busy == false && var:replan_pending == false && var:sop_transfer_sop_state == initial && var:sop_build_sop_state == initial";

fn sop_done(sop: &str) -> String {
    format!("var:{sop}_sop_state == completed")
}

impl Move {
    fn sop(&self) -> &'static str {
        match self {
            Move::Transfer { .. } => TRANSFER_SOP,
            Move::Build { .. } => BUILD_SOP,
        }
    }

    fn start_runner_actions(&self) -> Vec<String> {
        let (mut actions, label) = match self {
            Move::Transfer { die, pick, place, release, label } => (
                vec![
                    format!("var:transfer_die <- {die}"),
                    format!("var:transfer_pick <- {pick}"),
                    format!("var:transfer_pick_transit <- {}", transit_frame(pick)),
                    format!("var:transfer_place <- {place}"),
                    format!("var:transfer_place_transit <- {}", transit_frame(place)),
                    format!("var:transfer_release <- {release}"),
                ],
                label,
            ),
            Move::Build { die, place, label } => (
                vec![
                    format!("var:transfer_die <- {die}"),
                    format!("var:transfer_place <- {place}"),
                    format!("var:transfer_place_transit <- {}", transit_frame(place)),
                ],
                label,
            ),
        };
        actions.extend([
            format!("var:transfer_task <- \"{label}\""),
            format!("var:kpi_current_task <- \"{label}\""),
            "var:transfer_busy <- true".to_string(),
            format!("var:{}_sop_enabled <- true", self.sop()),
        ]);
        actions
    }
}

/// Puts the skills back to rest after an SOP ended, whichever way.
fn release_skills() -> Vec<String> {
    vec![
        format!("var:{TRANSFER_SOP}_sop_state <- initial"),
        format!("var:{BUILD_SOP}_sop_state <- initial"),
        "var:transfer_busy <- false".to_string(),
        "var:kpi_current_task <- \"-\"".to_string(),
    ]
}

fn finish_runner_actions(extra: &[String]) -> Vec<String> {
    let mut actions = release_skills();
    actions.push("var:kpi_run_transfers += 1".to_string());
    actions.extend(extra.iter().cloned());
    actions
}

/// The model's picture of the cell is suspect: look again (world_changed) and
/// do not start anything else from the old plan (replan_pending).
fn look_again(extra: &[&str]) -> Vec<String> {
    let mut actions = release_skills();
    actions.push("var:world_changed <- true".to_string());
    actions.push("var:replan_pending <- true".to_string());
    actions.extend(extra.iter().map(|a| a.to_string()));
    actions
}

/// The SOP failed - a motion the robot could not make, or a grip that closed
/// on nothing because the die was not where the model thought.
fn sop_failed(name: &str, sop: &str, state: &State) -> Transition {
    parse(&format!("{sop}_failed_{name}"), "true", &format!("var:{sop}_sop_state == fatal"), &[], &look_again(&[]), state)
}

/// What happens when a planned step has waited too long for its precondition:
/// the plan was made for a cell that no longer exists. The operation times
/// out, goes fatal, fails the plan - and this asks the goal runner to plan the
/// same goal again, from the state as it is now.
///
/// It is also how a disturbance stops a plan: the resync transition sets
/// replan_pending, which holds every planned step's runner guard false.
pub fn stale_plan_transitions(name: &str, config: &Config, state: &State) -> Vec<Transition> {
    vec![parse(
        &format!("stale_{name}"),
        "true",
        "true",
        &[],
        &[
            "var:replan_pending <- false".to_string(),
            format!("var:{}_replan_for_same_goal <- true", config.sp_id),
            "var:kpi_run_replans += 1".to_string(),
        ],
        state,
    )]
}

/// A planned operation that runs one SOP once.
///
/// `effects` are what the planner assumes and what the operation records when
/// the SOP completes; `on_completion` are runtime-only extras.
fn task(name: &str, config: &Config, guard: &str, effects: &[String], mv: Move, on_completion: &[String], state: &State) -> Operation {
    let sop = mv.sop();
    Operation::new(
        name,
        Some(PLANNED_STEP_TIMEOUT_MS),
        Some(config.stale_step_ms),
        None,
        None,
        false,
        vec![parse(&format!("start_{name}"), guard, READY, &[], &mv.start_runner_actions(), state)],
        vec![parse(&format!("complete_{name}"), "true", &sop_done(sop), effects, &finish_runner_actions(on_completion), state)],
        vec![sop_failed(name, sop, state)],
        stale_plan_transitions(name, config, state),
        vec![],
        vec![],
    )
}

/// The feeder may be emptied one more die only while no die the order wants is
/// lying in the arena - those go to a pad or the tower first - or when there is
/// no pad left to put one on.
fn fetch_allowed(config: &Config) -> String {
    let arena_clear = config
        .dice
        .iter()
        .map(|x| format!("(var:{x}_in_arena == false || var:{x}_wanted_face == 0)"))
        .collect::<Vec<_>>()
        .join(" && ");
    let pads_full = (1..=config.pads).map(|j| format!("var:pad_{j} != none")).collect::<Vec<_>>().join(" && ");
    format!("(({arena_clear}) || ({pads_full}))")
}

/// Every planned operation for this cell.
pub fn planned_operations(config: &Config, state: &State) -> Vec<Operation> {
    let levels = config.levels();
    let mut ops = vec![];

    // A level is "the top" when the one above it is empty, or there is none.
    let is_top = |k: usize| -> String {
        if k < levels { format!(" && var:level_{} == none", k + 1) } else { String::new() }
    };
    // And a die can go on level k when the one below it is taken.
    let on_something = |k: usize| -> String {
        if k > 1 { format!(" && var:level_{} != none", k - 1) } else { String::new() }
    };
    let fetch_allowed = fetch_allowed(config);

    for die in &config.dice {
        let d = die.as_str();

        // --- the next die out of the feeder, hoping it is this one.
        let name = format!("fetch_{d}");
        let mv = Move::Transfer {
            die: UNKNOWN_DIE.to_string(),
            pick: FEEDER_NEXT_FRAME.to_string(),
            place: ARENA_DROP_FRAME.to_string(),
            release: "throw",
            label: format!("fetch {d}?"),
        };
        let landed = sop_done(TRANSFER_SOP);
        ops.push(Operation::new(
            &name,
            Some(PLANNED_STEP_TIMEOUT_MS),
            Some(config.stale_step_ms),
            None,
            None,
            false,
            vec![parse(
                &format!("start_{name}"),
                &format!("var:{d}_seen == false && var:{d}_wanted_face != 0 && {fetch_allowed}"),
                READY,
                &[],
                &mv.start_runner_actions(),
                state,
            )],
            vec![parse(
                &format!("complete_{name}"),
                "true",
                &format!("{landed} && var:{d}_in_arena_measured == true"),
                &[format!("var:{d}_seen <- true"), format!("var:{d}_in_arena <- true")],
                &finish_runner_actions(&["var:kpi_run_throws += 1".to_string()]),
                state,
            )],
            vec![
                // The camera says it is another colour: take that in and replan.
                parse(
                    &format!("other_colour_{name}"),
                    "true",
                    &format!("{landed} && var:{d}_in_arena_measured == false"),
                    &[],
                    &look_again(&["var:kpi_run_transfers += 1", "var:kpi_run_throws += 1", "var:kpi_run_other_colour += 1"]),
                    state,
                ),
                sop_failed(&name, TRANSFER_SOP, state),
            ],
            stale_plan_transitions(&name, config, state),
            vec![],
            vec![],
        ));

        for k in 1..=levels {
            // --- from the arena onto level k, turned on the way.
            let name = format!("stack_{d}_on_level_{k}");
            ops.push(task(
                &name,
                config,
                &format!("var:{d}_in_arena == true && var:order_{k} == {d} && var:level_{k} == none{}", on_something(k)),
                &[
                    format!("var:level_{k} <- {d}"),
                    format!("var:level_{k}_face <- var:order_{k}_face"),
                    format!("var:{d}_in_arena <- false"),
                ],
                Move::Build { die: d.to_string(), place: level_frame(k), label: format!("stack {d} on level {k}") },
                &[],
                state,
            ));

            // --- throw off the top of the tower, into the arena.
            let name = format!("throw_{d}_from_level_{k}");
            ops.push(task(
                &name,
                config,
                &format!("var:level_{k} == {d}{}", is_top(k)),
                &[format!("var:level_{k} <- none"), format!("var:level_{k}_face <- 0"), format!("var:{d}_in_arena <- true")],
                Move::Transfer {
                    die: d.to_string(),
                    pick: level_frame(k),
                    place: ARENA_DROP_FRAME.to_string(),
                    release: "throw",
                    label: format!("throw {d} off level {k}"),
                },
                &["var:kpi_run_throws += 1".to_string()],
                state,
            ));
        }

        for j in 1..=config.pads {
            // --- park a die the order wants, but not yet.
            let name = format!("park_{d}_on_pad_{j}");
            ops.push(task(
                &name,
                config,
                &format!("var:{d}_in_arena == true && var:{d}_wanted_face != 0 && var:pad_{j} == none"),
                &[format!("var:pad_{j} <- {d}"), format!("var:{d}_in_arena <- false")],
                Move::Transfer {
                    die: d.to_string(),
                    pick: die_frame(d),
                    place: pad_frame(j),
                    release: "place",
                    label: format!("park {d} on pad {j}"),
                },
                &[],
                state,
            ));

            // --- clear a pad of a die the order no longer wants.
            let name = format!("throw_{d}_from_pad_{j}");
            ops.push(task(
                &name,
                config,
                &format!("var:pad_{j} == {d} && var:{d}_wanted_face == 0"),
                &[format!("var:pad_{j} <- none"), format!("var:{d}_in_arena <- true")],
                Move::Transfer {
                    die: d.to_string(),
                    pick: pad_frame(j),
                    place: ARENA_DROP_FRAME.to_string(),
                    release: "throw",
                    label: format!("throw {d} off pad {j}"),
                },
                &["var:kpi_run_throws += 1".to_string()],
                state,
            ));
        }
    }

    // Location-to-location moves don't care which die it is: the die travels
    // with the variables.
    for k in 1..=levels {
        for j in 1..=config.pads {
            let name = format!("unstack_level_{k}_to_pad_{j}");
            ops.push(task(
                &name,
                config,
                &format!("var:level_{k} != none{} && var:pad_{j} == none", is_top(k)),
                &[format!("var:pad_{j} <- var:level_{k}"), format!("var:level_{k} <- none"), format!("var:level_{k}_face <- 0")],
                Move::Transfer {
                    die: format!("var:level_{k}"),
                    pick: level_frame(k),
                    place: pad_frame(j),
                    release: "place",
                    label: format!("park level {k} on pad {j}"),
                },
                &[],
                state,
            ));

            let name = format!("stack_pad_{j}_on_level_{k}");
            ops.push(task(
                &name,
                config,
                &format!("var:pad_{j} != none && var:pad_{j} == var:order_{k} && var:level_{k} == none{}", on_something(k)),
                &[
                    format!("var:level_{k} <- var:pad_{j}"),
                    format!("var:level_{k}_face <- var:order_{k}_face"),
                    format!("var:pad_{j} <- none"),
                ],
                Move::Build { die: format!("var:pad_{j}"), place: level_frame(k), label: format!("stack pad {j} on level {k}") },
                &[],
                state,
            ));
        }
    }

    ops
}
