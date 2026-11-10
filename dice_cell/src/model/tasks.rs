//! The planned operations: what the cell *can* do, as the planner sees it.
//!
//! Six templates, instantiated per die, level and pad:
//!
//! | Template | From | To | What the planner assumes |
//! |---|---|---|---|
//! | `get_face_{die}` | feeder / arena | arena | it ends up showing the face the order wants |
//! | `stack_{die}_on_level_{k}` | feeder / arena | tower level k | - |
//! | `throw_{die}_from_level_{k}` | top of the tower | arena | nothing about the face |
//! | `throw_{die}_from_pad_{j}` | pad j | arena | nothing about the face |
//! | `unstack_level_{k}_to_pad_{j}` | top of the tower | pad j | the face is kept |
//! | `stack_pad_{j}_on_level_{k}` | pad j | tower level k | the face is kept |
//!
//! None of them names a sequence. Which dice need a new face, in what order,
//! whether a die that happens to show the right face can skip it, what to take
//! off a wrong tower and where to park it - all of that is the planner's.
//!
//! `get_face_{die}` is "throw it once, then flip it until it shows the wanted
//! face". A die still in the feeder is thrown into the arena (the transfer SOP);
//! a die already in the arena is flipped a quarter at the flip station (the flip
//! SOP). Whichever it was, an attempt that leaves the wrong face up *fails*, and
//! the failure's retry starts the operation over - by then the die is in the
//! arena, so the retry flips it. A face on a side is one flip away, the one
//! underneath two, so after the throw it takes at most two more attempts, and
//! the planner may count on the outcome. The two ways of starting are two
//! preconditions of the one operation, so to the planner it is one step, as a
//! throw used to be.
//!
//! The throws that clear a die off the tower or a pad are single throws, and the
//! planner is told the face is unknown afterwards - which is what makes it
//! prefer parking a die that already shows the right face over throwing it.
//! If such a throw happens to land on the wanted face anyway, the plan's next
//! `get_face_{die}` no longer applies, goes stale, and the goal is replanned
//! without it (see [`stale_plan_transitions`]).

use micro_sp::*;

use super::skills::{FLIP_SOP, TRANSFER_SOP};
use crate::{ARENA_DROP_FRAME, Config, die_frame, level_frame, pad_frame, transit_frame};

/// The throw, then at most two flips - and one to spare.
const GET_FACE_RETRIES: i64 = 3;

/// A planned step never times out while its transfer is running: every step
/// of the SOP has its own, shorter timeout, so the SOP always ends - and only
/// then may the step end. A step that gave up first would leave the SOP
/// running with nobody to collect it, and the transfer skill locked.
const PLANNED_STEP_TIMEOUT_MS: i64 = 24 * 3_600_000;

/// The six templates, for the GUI's Production tab.
pub const TEMPLATE_COUNT: i64 = 6;

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

/// What a planned operation hands the transfer SOP.
struct Transfer {
    /// The die's id, or `var:...` naming the location variable that holds it.
    die: String,
    pick: String,
    pick_transit: String,
    place: String,
    place_transit: String,
    release: &'static str,
    label: String,
}

/// The runner guard every planned operation starts on: the transfer skill is
/// free, and nothing has told us the plan is stale.
const READY: &str = "var:transfer_busy == false && var:replan_pending == false && var:sop_transfer_sop_state == initial && var:sop_flip_sop_state == initial";
const SOP_DONE: &str = "var:sop_transfer_sop_state == completed";
const FLIP_DONE: &str = "var:sop_flip_sop_state == completed";

fn start_runner_actions(t: &Transfer) -> Vec<String> {
    vec![
        format!("var:transfer_die <- {}", t.die),
        format!("var:transfer_pick <- {}", t.pick),
        format!("var:transfer_pick_transit <- {}", t.pick_transit),
        format!("var:transfer_place <- {}", t.place),
        format!("var:transfer_place_transit <- {}", t.place_transit),
        format!("var:transfer_release <- {}", t.release),
        format!("var:transfer_task <- \"{}\"", t.label),
        format!("var:kpi_current_task <- \"{}\"", t.label),
        "var:transfer_busy <- true".to_string(),
        format!("var:{TRANSFER_SOP}_sop_enabled <- true"),
    ]
}

/// What a planned operation hands the flip SOP: just the die.
fn start_flip_runner_actions(die: &str) -> Vec<String> {
    vec![
        format!("var:transfer_die <- {die}"),
        format!("var:transfer_task <- \"flip {die}\""),
        format!("var:kpi_current_task <- \"flip {die}\""),
        "var:transfer_busy <- true".to_string(),
        format!("var:{FLIP_SOP}_sop_enabled <- true"),
    ]
}

fn finish_runner_actions(extra: &[String]) -> Vec<String> {
    let mut actions = vec![
        format!("var:{TRANSFER_SOP}_sop_state <- initial"),
        format!("var:{FLIP_SOP}_sop_state <- initial"),
        "var:transfer_busy <- false".to_string(),
        "var:kpi_current_task <- \"-\"".to_string(),
        "var:kpi_run_transfers += 1".to_string(),
    ];
    actions.extend(extra.iter().cloned());
    actions
}

/// After a single throw the camera says what came up. The planner was told
/// "unknown" (face 0); at runtime the measured face replaces that.
fn single_throw_runner_actions(die: &str) -> Vec<String> {
    vec![
        format!("var:{die}_face <- var:{die}_face_measured"),
        "var:kpi_run_throws += 1".to_string(),
    ]
}

/// The SOP failed - a motion the robot could not make, or a grip that closed
/// on nothing because the die was not where the model thought. Either way the
/// model's picture of the cell is suspect: look again (world_changed) and do
/// not start anything else from the old plan (replan_pending).
fn sop_failed(name: &str, sop: &str, state: &State) -> Transition {
    parse(
        &format!("{sop}_failed_{name}"),
        "true",
        &format!("var:{sop}_sop_state == fatal"),
        &[],
        &[
            format!("var:{sop}_sop_state <- initial"),
            "var:transfer_busy <- false".to_string(),
            "var:kpi_current_task <- \"-\"".to_string(),
            "var:world_changed <- true".to_string(),
            "var:replan_pending <- true".to_string(),
        ],
        state,
    )
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

/// A planned operation that runs the transfer SOP once.
///
/// `effects` are what the planner assumes and what the operation records when
/// the SOP completes; `on_completion` are runtime-only extras, applied after
/// the effects (so they can correct one with a measurement).
fn task(
    name: &str,
    config: &Config,
    guard: &str,
    effects: &[String],
    transfer: Transfer,
    on_completion: &[String],
    state: &State,
) -> Operation {
    Operation::new(
        name,
        Some(PLANNED_STEP_TIMEOUT_MS),
        Some(config.stale_step_ms),
        None,
        None,
        false,
        vec![parse(&format!("start_{name}"), guard, READY, &[], &start_runner_actions(&transfer), state)],
        vec![parse(
            &format!("complete_{name}"),
            "true",
            SOP_DONE,
            effects,
            &finish_runner_actions(on_completion),
            state,
        )],
        vec![sop_failed(name, TRANSFER_SOP, state)],
        stale_plan_transitions(name, config, state),
        vec![],
        vec![],
    )
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

    for die in &config.dice {
        let d = die.as_str();

        // --- get the wanted face up: throw it once, then flip it until it shows.
        let name = format!("get_face_{d}");
        let needs_face = format!("var:{d}_loose == true && var:{d}_wanted_face != 0 && var:{d}_face != var:{d}_wanted_face");
        let shows_it = format!("var:{d}_face_measured == var:{d}_wanted_face");
        let shows_not = format!("var:{d}_face_measured != var:{d}_wanted_face");
        // What the planner assumes, and what a successful attempt records.
        let got_it = [format!("var:{d}_face <- var:{d}_wanted_face"), format!("var:{d}_in_arena <- true")];
        // An attempt that left the wrong face up: record what it did show, put
        // the skills back to rest, and fail - which retries, with a flip.
        let try_again = |extra: &[&str]| -> Vec<String> {
            let mut actions = vec![
                format!("var:{d}_face <- var:{d}_face_measured"),
                format!("var:{d}_in_arena <- true"),
                format!("var:{TRANSFER_SOP}_sop_state <- initial"),
                format!("var:{FLIP_SOP}_sop_state <- initial"),
                "var:transfer_busy <- false".to_string(),
                "var:kpi_current_task <- \"-\"".to_string(),
            ];
            actions.extend(extra.iter().map(|a| a.to_string()));
            actions
        };
        ops.push(Operation::new(
            &name,
            Some(PLANNED_STEP_TIMEOUT_MS),
            Some(config.stale_step_ms),
            Some(GET_FACE_RETRIES),
            None,
            false,
            vec![
                // Still in the feeder: its one throw.
                parse(
                    &format!("start_throw_{name}"),
                    &needs_face,
                    &format!("{READY} && var:{d}_in_arena == false"),
                    &[],
                    &start_runner_actions(&Transfer {
                        die: d.to_string(),
                        pick: die_frame(d),
                        pick_transit: transit_frame(&die_frame(d)),
                        place: ARENA_DROP_FRAME.to_string(),
                        place_transit: transit_frame(ARENA_DROP_FRAME),
                        release: "throw",
                        label: format!("throw {d}"),
                    }),
                    state,
                ),
                // Already in the arena: a flip.
                parse(
                    &format!("start_flip_{name}"),
                    &needs_face,
                    &format!("{READY} && var:{d}_in_arena == true"),
                    &[],
                    &start_flip_runner_actions(d),
                    state,
                ),
            ],
            vec![
                parse(
                    &format!("complete_throw_{name}"),
                    "true",
                    &format!("{SOP_DONE} && {shows_it}"),
                    &got_it,
                    &finish_runner_actions(&["var:kpi_run_throws += 1".to_string()]),
                    state,
                ),
                parse(
                    &format!("complete_flip_{name}"),
                    "true",
                    &format!("{FLIP_DONE} && {shows_it}"),
                    &got_it,
                    &finish_runner_actions(&["var:kpi_run_flips += 1".to_string()]),
                    state,
                ),
            ],
            vec![
                parse(
                    &format!("wrong_face_after_throw_{name}"),
                    "true",
                    &format!("{SOP_DONE} && {shows_not}"),
                    &[],
                    &try_again(&["var:kpi_run_transfers += 1", "var:kpi_run_throws += 1", "var:kpi_run_wrong_faces += 1"]),
                    state,
                ),
                // The face that was underneath takes two flips.
                parse(
                    &format!("one_more_flip_{name}"),
                    "true",
                    &format!("{FLIP_DONE} && {shows_not}"),
                    &[],
                    &try_again(&["var:kpi_run_transfers += 1", "var:kpi_run_flips += 1"]),
                    state,
                ),
                sop_failed(&name, TRANSFER_SOP, state),
                sop_failed(&name, FLIP_SOP, state),
            ],
            stale_plan_transitions(&name, config, state),
            vec![],
            vec![],
        ));

        for k in 1..=levels {
            // --- stack: loose die, already showing its face, onto level k.
            let name = format!("stack_{d}_on_level_{k}");
            ops.push(task(
                &name,
                config,
                &format!(
                    "var:{d}_loose == true && var:order_{k} == {d} && var:{d}_face == var:order_{k}_face && var:level_{k} == none{}",
                    on_something(k)
                ),
                &[
                    format!("var:level_{k} <- {d}"),
                    format!("var:level_{k}_face <- var:{d}_face"),
                    format!("var:{d}_loose <- false"),
                    format!("var:{d}_in_arena <- false"),
                ],
                Transfer {
                    die: d.to_string(),
                    pick: die_frame(d),
                    pick_transit: transit_frame(&die_frame(d)),
                    place: level_frame(k),
                    place_transit: transit_frame(&level_frame(k)),
                    release: "place",
                    label: format!("stack {d} on level {k}"),
                },
                &[],
                state,
            ));

            // --- throw off the top of the tower.
            let name = format!("throw_{d}_from_level_{k}");
            ops.push(task(
                &name,
                config,
                &format!("var:level_{k} == {d}{}", is_top(k)),
                &[
                    format!("var:level_{k} <- none"),
                    format!("var:level_{k}_face <- 0"),
                    format!("var:{d}_loose <- true"),
                    format!("var:{d}_in_arena <- true"),
                    // Unknown to the planner; the camera fills it in at runtime.
                    format!("var:{d}_face <- 0"),
                ],
                Transfer {
                    die: d.to_string(),
                    pick: level_frame(k),
                    pick_transit: transit_frame(&level_frame(k)),
                    place: ARENA_DROP_FRAME.to_string(),
                    place_transit: transit_frame(ARENA_DROP_FRAME),
                    release: "throw",
                    label: format!("throw {d} off level {k}"),
                },
                &single_throw_runner_actions(d),
                state,
            ));
        }

        for j in 1..=config.pads {
            // --- throw a parked die.
            let name = format!("throw_{d}_from_pad_{j}");
            ops.push(task(
                &name,
                config,
                &format!("var:pad_{j} == {d}"),
                &[
                    format!("var:pad_{j} <- none"),
                    format!("var:pad_{j}_face <- 0"),
                    format!("var:{d}_loose <- true"),
                    format!("var:{d}_in_arena <- true"),
                    // Unknown to the planner; the camera fills it in at runtime.
                    format!("var:{d}_face <- 0"),
                ],
                Transfer {
                    die: d.to_string(),
                    pick: pad_frame(j),
                    pick_transit: transit_frame(&pad_frame(j)),
                    place: ARENA_DROP_FRAME.to_string(),
                    place_transit: transit_frame(ARENA_DROP_FRAME),
                    release: "throw",
                    label: format!("throw {d} off pad {j}"),
                },
                &single_throw_runner_actions(d),
                state,
            ));
        }
    }

    // Location-to-location moves don't care which die it is: the die and its
    // face travel with the variables.
    for k in 1..=levels {
        for j in 1..=config.pads {
            let name = format!("unstack_level_{k}_to_pad_{j}");
            ops.push(task(
                &name,
                config,
                &format!("var:level_{k} != none{} && var:pad_{j} == none", is_top(k)),
                &[
                    format!("var:pad_{j} <- var:level_{k}"),
                    format!("var:pad_{j}_face <- var:level_{k}_face"),
                    format!("var:level_{k} <- none"),
                    format!("var:level_{k}_face <- 0"),
                ],
                Transfer {
                    die: format!("var:level_{k}"),
                    pick: level_frame(k),
                    pick_transit: transit_frame(&level_frame(k)),
                    place: pad_frame(j),
                    place_transit: transit_frame(&pad_frame(j)),
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
                &format!(
                    "var:pad_{j} != none && var:pad_{j} == var:order_{k} && var:pad_{j}_face == var:order_{k}_face && var:level_{k} == none{}",
                    on_something(k)
                ),
                &[
                    format!("var:level_{k} <- var:pad_{j}"),
                    format!("var:level_{k}_face <- var:pad_{j}_face"),
                    format!("var:pad_{j} <- none"),
                    format!("var:pad_{j}_face <- 0"),
                ],
                Transfer {
                    die: format!("var:pad_{j}"),
                    pick: pad_frame(j),
                    pick_transit: transit_frame(&pad_frame(j)),
                    place: level_frame(k),
                    place_transit: transit_frame(&level_frame(k)),
                    release: "place",
                    label: format!("stack pad {j} on level {k}"),
                },
                &[],
                state,
            ));
        }
    }

    ops
}
