//! The planner against the model, with no Redis and no robot: does it find the
//! plans the demo relies on, are they the short ones, and is it fast enough?
//!
//! micro_sp's planner is a breadth-first search with a depth limit of 20 steps
//! and a 5 s budget (planner_ticker.rs), so both are checked here with the same
//! numbers.

use dice_cell::model::goal::tower_goal;
use dice_cell::model::offline_state;
use dice_cell::order::{order_state, parse_order};
use dice_cell::*;
use micro_sp::*;

const MAX_DEPTH: usize = 20;
const BUDGET_MS: u64 = 5000;

struct Scene {
    config: Config,
    model: Model,
    state: State,
}

impl Scene {
    fn new(n_dice: usize, pads: usize) -> Scene {
        let config = Config::for_tests(n_dice, pads);
        let (model, state) = offline_state(&config);
        Scene { config, model, state }
    }

    fn set(&mut self, name: &str, value: SPValue) {
        self.state = self.state.update(name, value);
    }

    /// Seen, and lying in the arena.
    fn in_arena(&mut self, die: &str) {
        self.set(&format!("{die}_seen"), true.to_spvalue());
        self.set(&format!("{die}_in_arena"), true.to_spvalue());
    }

    fn on_level(&mut self, k: usize, die: &str, face: i64) {
        self.set(&format!("{die}_seen"), true.to_spvalue());
        self.set(&format!("level_{k}"), die.to_string().to_spvalue());
        self.set(&format!("level_{k}_face"), face.to_spvalue());
    }

    fn on_pad(&mut self, j: usize, die: &str) {
        self.set(&format!("{die}_seen"), true.to_spvalue());
        self.set(&format!("pad_{j}"), die.to_string().to_spvalue());
    }

    fn order(&mut self, text: &str) {
        let levels = parse_order(text, &self.config).unwrap();
        self.state = self.state.extend(order_state(&levels, &self.config), true);
    }

    fn plan(&self) -> PlanningResult {
        let goal = pred_parser::pred(&tower_goal(&self.config), &self.state).unwrap();
        bfs_operation_planner(&self.state, &goal, &self.model.operations, MAX_DEPTH, "test", BUDGET_MS)
    }
}

fn steps(result: &PlanningResult) -> Vec<String> {
    result.plan.iter().map(|s| s.trim_start_matches("op_").to_string()).collect()
}

fn report(name: &str, result: &PlanningResult) {
    println!("{name}: found={} length={} time={:?}", result.found, result.length, result.time);
    for (i, step) in steps(result).iter().enumerate() {
        println!("    {:>2}. {step}", i + 1);
    }
}

// A scene starts with every die unseen: in the feeder, nobody knows where.

#[test]
fn six_unseen_dice_are_fetched_in_the_order_they_are_needed() {
    let mut scene = Scene::new(6, 3);
    scene.order("red:6 green:5 blue:4 yellow:3 purple:2 orange:1");
    let result = scene.plan();
    report("six unseen", &result);
    assert!(result.found);
    // A hopeful plan: each fetch assumes the next die out of the feeder is the
    // one the tower needs next, and each stack turns it on the way.
    assert_eq!(result.length, 12);
    let plan = steps(&result);
    assert_eq!(plan[0], "fetch_red");
    assert_eq!(plan[1], "stack_red_on_level_1");
    assert!(result.time.as_millis() < BUDGET_MS as u128);
}

#[test]
fn a_die_that_came_out_too_early_waits_on_a_pad() {
    // The plan hoped for red, but the feeder gave green.
    let mut scene = Scene::new(6, 3);
    scene.in_arena("green");
    scene.order("red:6 green:5 blue:4");
    let result = scene.plan();
    report("green too early", &result);
    assert!(result.found);
    // Park green, fetch red, stack red, green from the pad, fetch blue, stack it.
    assert_eq!(result.length, 6);
    let plan = steps(&result);
    assert!(plan[0].starts_with("park_green_on_pad_"), "{plan:?}");
    assert!(plan.iter().any(|s| s.starts_with("stack_pad_") && s.ends_with("_on_level_2")));
}

#[test]
fn a_die_the_order_does_not_want_stays_in_the_arena() {
    let mut scene = Scene::new(6, 3);
    scene.in_arena("yellow");
    scene.order("red:6 green:5");
    let result = scene.plan();
    report("yellow not wanted", &result);
    assert!(result.found);
    assert_eq!(result.length, 4);
    assert!(steps(&result).iter().all(|s| !s.contains("yellow")));
}

#[test]
fn with_every_pad_taken_the_feeder_is_used_anyway() {
    // Three dice parked for later, and the die for level 2 in the arena.
    let mut scene = Scene::new(6, 3);
    scene.on_pad(1, "blue");
    scene.on_pad(2, "yellow");
    scene.on_pad(3, "purple");
    scene.in_arena("green");
    scene.order("red:1 green:2 blue:3 yellow:4 purple:5");
    let result = scene.plan();
    report("pads full", &result);
    assert!(result.found);
    // Fetch red, then everything stacks: six steps.
    assert_eq!(result.length, 6);
    assert_eq!(steps(&result)[0], "fetch_red");
}

#[test]
fn a_knocked_over_tower_is_rebuilt_straight_from_the_arena() {
    let mut scene = Scene::new(6, 3);
    for die in scene.config.dice.clone() {
        scene.in_arena(&die);
    }
    scene.order("red:6 green:5 blue:4 yellow:3 purple:2 orange:1");
    let result = scene.plan();
    report("knocked over", &result);
    assert!(result.found);
    // One build per die - the flips happen inside it.
    assert_eq!(result.length, 6);
    assert!(steps(&result).iter().all(|s| s.starts_with("stack_")));
}

#[test]
fn a_wrong_level_two_is_taken_down_and_the_top_is_kept() {
    // red:6 / blue:2 / green:3 is standing; the order wants green under blue.
    let mut scene = Scene::new(6, 3);
    scene.on_level(1, "red", 6);
    scene.on_level(2, "blue", 2);
    scene.on_level(3, "green", 3);
    scene.order("red:6 green:3 blue:2");
    let result = scene.plan();
    report("swap two and three", &result);
    assert!(result.found);
    // Take green and blue off, stack green, stack blue.
    assert_eq!(result.length, 4);
    assert!(steps(&result).iter().all(|s| !s.starts_with("fetch")));
}

#[test]
fn a_changed_order_reuses_what_it_can() {
    // A full five-high tower, and an order that keeps the bottom three.
    let mut scene = Scene::new(6, 3);
    scene.on_level(1, "red", 6);
    scene.on_level(2, "green", 5);
    scene.on_level(3, "blue", 4);
    scene.on_level(4, "yellow", 3);
    scene.on_level(5, "purple", 2);
    scene.order("red:6 green:5 blue:4 orange:6 yellow:3");
    let result = scene.plan();
    report("new top", &result);
    assert!(result.found);
    // Purple comes off, yellow comes off, orange is fetched and stacked,
    // yellow goes back on top: five steps.
    assert_eq!(result.length, 5, "took {} steps", result.length);
    assert!(steps(&result).contains(&"fetch_orange".to_string()));
}

#[test]
fn the_worst_case_for_six_dice_still_plans_in_time() {
    // A wrong six-high tower and an order that reverses it: everything has to
    // come off, which is where the depth limit starts to show.
    let mut scene = Scene::new(6, 3);
    let tower = [("red", 1), ("green", 2), ("blue", 3), ("yellow", 4), ("purple", 5), ("orange", 6)];
    for (k, (die, face)) in tower.iter().enumerate() {
        scene.on_level(k + 1, die, *face);
    }
    scene.order("orange:6 purple:5 yellow:4 blue:3 green:2 red:1");
    let result = scene.plan();
    report("reverse the tower", &result);
    assert!(result.found);
    assert!(result.time.as_millis() < BUDGET_MS as u128);
}
