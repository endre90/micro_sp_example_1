# micro_sp example 1: the dice cell

A UR5 builds a tower of coloured dice to order. Every die has to show a given
face. A die arrives in the feeder showing whatever it shows; if that is wrong,
it is thrown into the arena once, and from then on its face is changed by
flipping it onto its side at the flip station - a quarter turn per flip, two
for the face that ended up underneath. The order can change while the tower is
being built, and an operator can knock the tower over, take a die away or throw
one at any moment.

The point is the control logic. Nobody wrote down a sequence for any of this.
The model is **six operation templates, two SOPs and five automatic
transitions**, written in [`micro_sp`](https://github.com/endre90/micro_sp).
The planner works out what to do from whatever the cell looks like at that
moment. The same model handles every situation below without a single extra
branch.

## Why planning, and not an SFC

Write it as an SFC or SOP and the branches multiply:

* which dice are in the feeder, in what order, and which of them already show
  the right face (and can skip the throw);
* every throw is a fork: right face, carry on; a face on a side, one flip;
  the face underneath, two flips;
* a tower that is already standing and is right up to some level, but wrong
  above it (what comes off, what can be parked on a pad with its face kept,
  what has to be thrown off and flipped);
* an order that changes halfway through, at any step;
* a disturbance at any step: the tower knocked over, a die taken off, a die
  thrown into the arena. Each of those leaves the cell in a different state,
  and every one of those states needs its own recovery branch.

Six dice, six faces, three pads and a tower of up to six levels give tens of
thousands of reachable situations. Here they all go through the same few
lines:

| Template | Moves a die from | to | The planner assumes |
|---|---|---|---|
| `get_face_{die}` | feeder / arena | arena | it ends up showing the face the order wants (thrown once if it is in the feeder, then flipped until it does) |
| `stack_{die}_on_level_{k}` | feeder / arena | tower level *k* | - |
| `throw_{die}_from_level_{k}` | top of the tower | arena | nothing about the face |
| `throw_{die}_from_pad_{j}` | pad *j* | arena | nothing about the face |
| `unstack_level_{k}_to_pad_{j}` | top of the tower | pad *j* | the face is kept |
| `stack_pad_{j}_on_level_{k}` | pad *j* | tower level *k* | the face is kept |

With six dice and three pads these expand to 132 planned operations. The
planner (micro_sp's breadth-first search) picks the shortest sequence. On this
machine that takes 10-400 ms for an ordinary order, and 3.4 s for the worst
case tested: reversing a full six-dice tower takes 15 steps, keeps three dice
on the pads with their faces and throws off and re-faces the other three.
`dice_cell/tests/planning.rs` holds these cases as tests.

How the work is split:

* **Plan** (task level): which die, where, in what order. This is
  `dice_cell/src/model/tasks.rs`.
* **SOPs** (skill level): the fixed motions, one SOP per way of handling a
  die. *Transfer* moves one die: travel, descend, grip, lift, travel, descend,
  release (place or throw), retract. *Flip* turns one die a quarter: grip it
  turned so the right face is on the right side, carry it to the flip
  station, tip the tool over in the air, set the die down on its side, take it
  again from above and put it back in the arena. Both keep to the same two
  rules, which are what keep the arm clear of the table, the tower and the
  dice: travel only at a fixed safe height (the TCP at 0.34 m, through a
  `*_transit` frame above each place) with joint moves, and go down to and up
  from a place only in a straight vertical line (`move_l`), the last 10 cm
  slowly. This is `dice_cell/src/model/skills.rs`. The planned operations hand
  them parameters; they never see the motions.
* **Automatic transitions**: when the camera reports a change the model did not
  make, they copy the camera's picture into the model's belief and stop the
  running plan at its next step. This is `dice_cell/src/model/reactions.rs`.
* **Goal**: "the tower is the order". It is written against the order
  variables, so it never changes. A new order changes what the goal means, and
  the running goal is simply replanned. This is
  `dice_cell/src/model/goal.rs`.

## Running it

Two ways, the same as campx_rita. Both publish the same ports, so **do not run
both at once**.

### Docker (nothing to install but Docker)

```bash
cd run/sim_docker
./run_in_docker.sh                           # build (cached), start, attach to a log grid
./run_in_docker.sh --order "red:6 green:5"   # ...and place an order straight away
./run_in_docker.sh --stop
```

The first run pulls the ROS and Rust images and compiles five Rust projects,
which takes several minutes and a few GB.

### Native (from checkouts you can edit)

```bash
cd run/sim_native
./install_natively.sh       # once: build tools, ROS 2, Rust + trunk, Docker, upstream clones, builds
./run_natively.sh           # 3x3 tmux grid; Redis runs in a container, the rest natively
./run_natively.sh --stop
```

All settings are in `run/sim_native/.env` and `run/sim_docker/.env`.

| What | Where |
|---|---|
| micro_sp GUI | <http://localhost:8080> |
| RViz (Docker) | <http://localhost:6081/vnc.html?autoconnect=1&resize=remote> |
| RViz (native) | a window on the desktop |

### The stack

| Process | What it is |
|---|---|
| `runner` (dice_cell) | The control system: the model, micro_sp's runners, the order handler |
| `cell` (dice_cell) | Everything physical except the robot: dice, gripper, camera, the operator's disturbances |
| [simple_robot_simulator](https://github.com/sequenceplanner/simple_robot_simulator) | The UR5, kinematically: IK against the transform tree, joints and link frames into Redis |
| [r2r_redis_visualization](https://github.com/sequenceplanner/r2r_redis_visualization) | Redis transforms and meshes to ROS 2 `/tf` and `/mesh_markers` |
| RViz 2 | The view; served to the browser over noVNC in the Docker run |
| [micro_sp_gui](https://github.com/endre90/micro_sp_gui) | State, goals, plans, logs and KPIs |
| Redis | The state everyone shares |

## Driving the demo

Everything is done from the GUI's **State** tab by writing one of two variables.

**`order`**: the tower you want, bottom level first, as `colour:face`:

```text
red:6 green:5 blue:4
yellow:1, purple:1 > orange:2
clear                       (no tower: everything comes off)
```

The order handler checks it (the result is in `order_status`), writes the order
variables and posts the tower goal. If a tower is already being built, the
running plan is stopped at its next step and replanned instead.

**`disturbance`**: what the operator does to the cell:

| Value | What happens |
|---|---|
| `knock_tower` | The whole tower tumbles into the arena, every die showing a new face |
| `steal_top` | The top die is taken off and put back in the feeder, face unchanged |
| `steal <colour>` | That die goes back to the feeder; anything above it falls into the arena |
| `bump <colour>` | That die is thrown into the arena; anything above it falls too |
| `reset` | Every die back in the feeder, in a new order, showing new faces |

`disturbance_status` says what was done. If the tower is knocked over after it
was finished, it is rebuilt without anyone asking: the last goal is replanned
from where the dice landed.

### A script for a demo

1. `order` = `red:6 green:5 blue:4 yellow:3`. Watch the plan (Goals tab): a die
   that already happens to show its face is not thrown. One that comes up
   wrong is flipped at the station - once, or twice if its face ended up
   underneath.
2. Halfway up, `order` = `red:6 yellow:3 blue:2`. The plan stops at the next
   step and a new one takes off what is wrong and reuses what is right.
3. Once it is done, `disturbance` = `knock_tower`. The cell rebuilds it with
   flips only - the dice are in the arena already - skipping any die that
   landed on the right face.
4. During a build, `disturbance` = `steal_top`, then `bump blue`.
5. `order` = `clear`.

The **Production** tab counts throws, wrong faces, flips, transfers, replans,
disturbances and orders, and shows the operation count against the number of
templates (`kpi_model_*`).

## Variables worth watching

| Variable | Meaning |
|---|---|
| `order`, `order_status` | The order, and whether it was taken |
| `order_k`, `order_k_face` | The order per level, as the model uses it |
| `level_k`, `level_k_face`, `pad_j`, `pad_j_face` | What the model believes stands where |
| `{colour}_loose`, `{colour}_face` | What it believes about a loose die (in the feeder or the arena) |
| `{colour}_in_arena` | The die has had its one throw: its face is changed by flipping from now on |
| `*_measured` | What the camera sees (written only by `cell`) |
| `transfer_task` | What the robot is doing right now (a transfer, or `flip <colour>`) |
| `world_changed`, `replan_pending` | A disturbance was seen; the running plan is being stopped |
| `tower_goal` | The goal predicate, for posting by hand from the Goals tab |

## How replanning works

micro_sp's goal runner replans "the same goal" when `{sp}_replan_for_same_goal`
is set, but only once the current plan has ended. The cell gets there in two
ways:

* **A throw or a flip comes up wrong.** `get_face_{die}` *fails*, and its
  failure retry starts it again - which, with the die now in the arena, is a
  flip. The cell picks the flip: the wanted face if it is on a side, otherwise
  any side face, and the retry after that brings up the wanted one. No replan
  is needed, because there is nothing to decide.
* **The world changed.** The cell raises `world_changed`, and an automatic
  transition copies the camera's reading into the belief and sets
  `replan_pending`. That holds back the next planned step. After
  `STALE_STEP_MS` (1 s) the step times out, its timeout transition asks for the
  replan, the plan fails and the goal is planned again from the cell as it now
  is. The step that is already under way always finishes first, so the robot
  is never stopped with a die in its hand. A plan step whose precondition no
  longer holds goes stale the same way, whatever the reason.
* **The plan ran out while it was being held back.** If the change happens
  during the last step, no later step is left to go stale. When a plan ends
  with `replan_pending` still set, the goal is replanned anyway. If the goal
  already holds, the planner says so and nothing moves.

The order handler and the automatic transitions never set
`{sp}_replan_for_same_goal` while a plan is executing. micro_sp's goal runner
acts on that flag immediately whenever it reads the goal as `initial`, which is
the case after any replan. Acting on it then would abandon a step whose
transfer is still running. Instead, they hold the plan back and let a stale
step, or the end of the plan, ask for the replan.

## Layout

| Path | What is in it |
|---|---|
| `dice_cell/` | The crate: `model/` (state, skills, tasks, reactions, goal), `order.rs`, `cell/` (the simulated cell), `bin/runner.rs`, `bin/cell.rs`, `tests/planning.rs` |
| `shared_folder/meshes`, `shared_folder/scenario` | Meshes (STL, mm) and the static scene, one frame per JSON file |
| `tools/generate_scene.py` | Generates both from one layout, including the flip station and the transit frames. The die size, the arena, the safe height and the frame names are mirrored in `dice_cell/src/layout.rs` |
| `docker/*/docker` | One build context per image |
| `run/sim_native`, `run/sim_docker` | The two launchers, their `.env` and compose files |

## Limits, honestly

* **The planner's depth limit is 20 steps** (and it has a 5 s budget); both are
  hard-coded in micro_sp's `planner_ticker`. Six dice fit, including the worst
  case above (15 steps). Larger cells would need a heuristic search, which this
  example makes a good case for.
* The robot is kinematic only: there is no collision checking at run time.
  The arm stays clear because of the route the SOPs take (safe height, vertical
  approaches); the paths were checked offline against the scene, and in a
  recorded run the closest any part of the arm or gripper came to the table, a
  wall, a plate or the flip station was a fingertip 2 mm above the arena floor
  while gripping.
* The motion needs the simple_robot_simulator in `native_deps` as it is in this
  repository: real straight-line `move_l`, smooth joint moves, tight IK with
  staged solving for large wrist turns. Upstream `master` approximates
  `move_l` in joint space and its IK gives up on large turns, so the Docker
  run, which builds the simulator from `SIM_GIT_REF`, needs those changes
  pushed (or `SIM_GIT_URL`/`SIM_GIT_REF` pointed at a branch that has them).
* The dice are not physically simulated. A throw picks a free spot in the arena,
  a random yaw and a random orientation, and animates the fall. A flip is
  exact: the die lands on its side the way the gripper held it.
* The camera is perfect: `*_measured` is the truth.

## Tests

```bash
cd dice_cell
cargo test --release        # planning against the real model, no Redis needed
```
