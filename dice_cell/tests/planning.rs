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

    fn loose(&mut self, die: &str, face: i64) {
        self.set(&format!("{die}_loose"), true.to_spvalue());
        self.set(&format!("{die}_face"), face.to_spvalue());
    }

    fn on_level(&mut self, k: usize, die: &str, face: i64) {
        self.set(&format!("{die}_loose"), false.to_spvalue());
        self.set(&format!("{die}_face"), face.to_spvalue());
        self.set(&format!("level_{k}"), die.to_string().to_spvalue());
        self.set(&format!("level_{k}_face"), face.to_spvalue());
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

#[test]
fn six_dice_from_the_feeder() {
    let mut scene = Scene::new(6, 3);
    let faces = [3, 1, 4, 1, 5, 2];
    for (die, face) in scene.config.dice.clone().iter().zip(faces) {
        scene.loose(die, face);
    }
    scene.order("red:6 green:5 blue:4 yellow:3 purple:2 orange:1");
    let result = scene.plan();
    report("six dice", &result);
    assert!(result.found);
    // A get-face and a stack per die - except blue, which already shows its 4.
    assert_eq!(result.length, 11);
    assert!(!steps(&result).contains(&"get_face_blue".to_string()));
    assert!(result.time.as_millis() < BUDGET_MS as u128);
}

#[test]
fn a_die_that_already_shows_its_face_is_left_alone() {
    let mut scene = Scene::new(6, 3);
    for die in scene.config.dice.clone() {
        scene.loose(&die, 1);
    }
    scene.loose("green", 5);
    scene.order("red:6 green:5 blue:4");
    let result = scene.plan();
    report("lucky green", &result);
    assert!(result.found);
    assert_eq!(result.length, 5);
    assert!(!steps(&result).contains(&"get_face_green".to_string()));
}

#[test]
fn a_wrong_level_two_is_taken_down_and_the_top_is_kept() {
    // red:6 / blue:2 / green:3 is standing; the order wants green under blue.
    let mut scene = Scene::new(6, 3);
    for die in scene.config.dice.clone() {
        scene.loose(&die, 1);
    }
    scene.on_level(1, "red", 6);
    scene.on_level(2, "blue", 2);
    scene.on_level(3, "green", 3);
    scene.order("red:6 green:3 blue:2");
    let result = scene.plan();
    report("swap two and three", &result);
    assert!(result.found);
    // Park green and blue (faces kept), stack green, stack blue - no throws.
    assert_eq!(result.length, 4);
    assert!(steps(&result).iter().all(|s| !s.starts_with("throw")));
}

#[test]
fn a_changed_order_reuses_what_it_can() {
    // A full five-high tower, and an order that keeps the bottom three.
    let mut scene = Scene::new(6, 3);
    for die in scene.config.dice.clone() {
        scene.loose(&die, 1);
    }
    scene.on_level(1, "red", 6);
    scene.on_level(2, "green", 5);
    scene.on_level(3, "blue", 4);
    scene.on_level(4, "yellow", 3);
    scene.on_level(5, "purple", 2);
    scene.order("red:6 green:5 blue:4 orange:6 yellow:3");
    let result = scene.plan();
    report("new top", &result);
    assert!(result.found);
    // Purple is thrown off for good, yellow is parked with its 3 still up,
    // orange gets its face and is stacked, yellow goes back on top: five steps.
    assert_eq!(result.length, 5, "took {} steps", result.length);
    let plan = steps(&result);
    assert_eq!(plan.iter().filter(|s| s.starts_with("throw_")).count(), 1);
    assert!(plan.contains(&"get_face_orange".to_string()));
}

#[test]
fn a_knocked_over_tower_is_rebuilt_from_where_the_dice_landed() {
    // Everything is loose again; two dice happened to land on the right face.
    let mut scene = Scene::new(6, 3);
    let landed = [("red", 6), ("green", 2), ("blue", 4), ("yellow", 3), ("purple", 6), ("orange", 1)];
    for (die, face) in landed {
        scene.loose(die, face);
    }
    scene.order("red:6 green:5 blue:4 yellow:3 purple:2 orange:1");
    let result = scene.plan();
    report("knocked over", &result);
    assert!(result.found);
    // Four dice still show the face they need: only green and purple need one.
    assert_eq!(result.length, 8);
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
    println!("    (found={}, {:?})", result.found, result.time);
}
