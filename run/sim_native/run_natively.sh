#!/bin/bash
#
# Starts the dice cell from local checkouts, one component per tmux pane.
#
# Redis runs in a container (docker-compose.yaml next to this file); everything
# else - the robot simulator, the visualization bridge, RViz, the runner, the
# simulated cell and the micro_sp GUI - is built and run from disk, so a change
# is: edit, Ctrl+C the pane, up arrow, enter.
#
#   ./run_natively.sh                 start the grid and attach
#   ./run_natively.sh --check         verify the prerequisites and exit
#   ./run_natively.sh --show          print the resolved paths and settings and exit
#   ./run_natively.sh --attach        re-attach to an already running grid
#   ./run_natively.sh --stop          kill the grid and stop Redis
#   ./run_natively.sh --clean         as --stop, and drop Redis's volume
#   ./run_natively.sh --order "..."   place this order once the runner is up
#   ./run_natively.sh --no-wait       start every pane at once, without the gates
#   ./run_natively.sh --rebuild-gui   rebuild and restart only the GUI pane
#
# Panes wait for what they need: everything for Redis, and the cell for the
# runner to have pushed the model's state (/tmp/runner_ready.flag).
#
# The dockerized twin is ../sim_docker/run_in_docker.sh. Both publish the same
# ports - do not run them at once.

set -uo pipefail

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    echo "Run this script, do not source it:  ./$(basename "${BASH_SOURCE[0]}") $*" >&2
    return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"

# shellcheck source=native_env.sh
source "$SCRIPT_DIR/native_env.sh"

SESSION_NAME="${SESSION_NAME:-dice_grid}"
ACTION="up"
USE_WAITS=1
ORDER_OVERRIDE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check)   ACTION="check" ;;
        --show)    ACTION="show" ;;
        --attach)  ACTION="attach" ;;
        --stop)    ACTION="stop" ;;
        --clean)   ACTION="clean" ;;
        --order)   ORDER_OVERRIDE="${2:?--order needs a value, e.g. \"red:6 green:5\"}"; shift ;;
        --order=*) ORDER_OVERRIDE="${1#*=}" ;;
        --no-wait) USE_WAITS=0 ;;
        --rebuild-gui) ACTION="rebuild-gui" ;;
        -h|--help) sed -n '2,25p' "$SCRIPT_PATH" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# --- Prerequisites ------------------------------------------------------------

MISSING=0

need() {
    printf '  \033[31mmissing\033[0m  %s\n' "$1" >&2
    [[ -n "$2" ]] && printf '           %s\n' "$2" >&2
    MISSING=1
}

check_prerequisites() {
    command -v tmux >/dev/null 2>&1 || need "tmux" "sudo apt install tmux"
    command -v cargo >/dev/null 2>&1 || need "the Rust toolchain (cargo)" "./install_natively.sh"
    docker compose version >/dev/null 2>&1 || need "docker compose" "./install_natively.sh - Redis runs in a container"
    docker info >/dev/null 2>&1 || need "a reachable Docker daemon" "start it, or add yourself to the 'docker' group"

    [[ -f "$ROS_SETUP" ]] || need "ROS 2 at $ROS_SETUP" "./install_natively.sh, or set ROS_SETUP in .env"
    if [[ -f "$ROS_SETUP" ]] && ! ( native_source_ros >/dev/null 2>&1 && command -v rviz2 >/dev/null 2>&1 ); then
        need "rviz2" "sudo apt install ros-$ROS_DISTRO-rviz2"
    fi

    [[ -f "$VIZ_DIR/Cargo.toml" ]] || need "r2r_redis_visualization at $VIZ_DIR" "./install_natively.sh --sources"
    [[ -f "$SIM_DIR/Cargo.toml" ]] || need "simple_robot_simulator at $SIM_DIR" "./install_natively.sh --sources"
    [[ -f "$UR_DESCRIPTION_DIR/urdf/$ROBOT_MODEL.urdf" ]] ||
        need "the $ROBOT_MODEL URDF in $UR_DESCRIPTION_DIR" "./install_natively.sh --sources (it ships with ur_redis_driver)"
    [[ -f "$DICE_CELL_DIR/Cargo.toml" ]] || need "dice_cell at $DICE_CELL_DIR" ""
    [[ -f "$MICRO_SP_GUI_DIR/Cargo.toml" ]] || need "micro_sp_gui at $MICRO_SP_GUI_DIR" "./install_natively.sh --sources"
    command -v trunk >/dev/null 2>&1 || need "trunk, which builds the GUI's wasm frontend" "./install_natively.sh --deps"
    rustup target list --installed 2>/dev/null | grep -qx wasm32-unknown-unknown ||
        need "the wasm32-unknown-unknown Rust target" "rustup target add wasm32-unknown-unknown"
    [[ -d "$MESHES_DIR" ]] || need "the meshes at $MESHES_DIR" "python3 tools/generate_scene.py"
    [[ -d "$SCENARIO_DIR" ]] || need "the scene at $SCENARIO_DIR" "python3 tools/generate_scene.py"
    [[ -f "$RVIZ_CONFIG" ]] || need "the RViz config at $RVIZ_CONFIG" "set RVIZ_CONFIG in .env"

    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q '_dice_docker$'; then
        printf '  \033[33mwarning\033[0m  the dockerized stack is running and owns the same ports\n' >&2
        printf '           stop it first: (cd ../sim_docker && ./run_in_docker.sh --stop)\n' >&2
    fi
    return $MISSING
}

GUI_PANE_TITLE="micro_sp GUI"

gui_command() {
    local setup="source \"$SCRIPT_DIR/native_env.sh\";"
    [[ "$USE_WAITS" == "0" ]] && setup="NATIVE_SKIP_GATES=1; $setup"
    printf '%s\n' "$setup
native_wait_for_redis;
cd \"\$MICRO_SP_GUI_DIR\";
mkdir -p \"\$MICRO_SP_GUI_FRAMES_DIR\";
export SP_ID=\"\$SP_INSTANCE_ID\";
native_trunk build --release && native_cargo run --release -p micro_sp_gui_server -- --bind \"0.0.0.0:\$MICRO_SP_GUI_PORT\" --frames-dir \"\$MICRO_SP_GUI_FRAMES_DIR\"
"
}

case "$ACTION" in
    show)  native_print_env; exit 0 ;;
    check)
        echo "Checking the native prerequisites..."
        if check_prerequisites; then echo "  everything the native run needs is in place."; exit 0; fi
        echo; echo "Run ./install_natively.sh to fix the above."; exit 1 ;;
    stop)
        echo "Stopping the grid and Redis..."
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        ( cd "$SCRIPT_DIR" && docker compose down )
        exit 0 ;;
    clean)
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        ( cd "$SCRIPT_DIR" && docker compose down -v )
        exit 0 ;;
    attach) exec tmux attach-session -t "$SESSION_NAME" ;;
    rebuild-gui)
        tmux has-session -t "$SESSION_NAME" 2>/dev/null ||
            { echo "No grid is running - start it with ./run_natively.sh." >&2; exit 1; }
        gui_pane="$(tmux list-panes -t "$SESSION_NAME:0" -F '#{pane_index}	#{pane_title}' |
            awk -F '\t' -v title="$GUI_PANE_TITLE" '$2 == title { print $1; exit }')"
        [[ -n "$gui_pane" ]] || { echo "The grid has no '$GUI_PANE_TITLE' pane." >&2; exit 1; }
        tmux respawn-pane -k -t "$SESSION_NAME:0.$gui_pane" -c "$SCRIPT_DIR"
        tmux select-pane -t "$SESSION_NAME:0.$gui_pane" -T "$GUI_PANE_TITLE"
        tmux send-keys -t "$SESSION_NAME:0.$gui_pane" "$(gui_command)" C-m
        exit 0 ;;
esac

if ! check_prerequisites; then
    echo >&2
    echo "Run ./install_natively.sh to fix the above, or ./run_natively.sh --check to re-check." >&2
    exit 1
fi

mkdir -p "$MICRO_SP_ACTIVITY_LOG_DIR"

# A fresh Redis, healthy before any pane starts. Done here rather than in a pane
# because the other panes gate on port 6379: a Redis still running from the last
# session would let them connect and seed state into an instance that the
# recreate is about to replace (the runner's seed then fails with "broken pipe"
# and every model variable is missing). --force-recreate because a container
# created before Docker last restarted (e.g. a reboot) can come back up with no
# network at all - loopback only, no published port.
echo "Starting a fresh Redis..."
if ! ( cd "$SCRIPT_DIR" && docker compose up -d --force-recreate --remove-orphans --wait ); then
    echo "Redis did not come up healthy - see: (cd $SCRIPT_DIR && docker compose logs redis)" >&2
    exit 1
fi
# A flag left behind by a previous run would let the cell start before the
# runner has seeded anything.
rm -f /tmp/runner_ready.flag

# --- Pane commands ------------------------------------------------------------
#
# Backslash-escaped `\$VAR` is resolved in the pane, after native_env.sh has
# been sourced there. A trailing newline (not ';') ends each command: tmux reads
# a ';' at the end of a send-keys argument as its own separator.

SETUP="source \"$SCRIPT_DIR/native_env.sh\";"
SETUP_ROS="$SETUP native_source_ros;"
if [[ "$USE_WAITS" == "0" ]]; then
    SETUP="NATIVE_SKIP_GATES=1; $SETUP"
    SETUP_ROS="NATIVE_SKIP_GATES=1; $SETUP_ROS"
fi

ORDER_EXPORT=""
[[ -n "$ORDER_OVERRIDE" ]] && ORDER_EXPORT="export INITIAL_ORDER=\"$ORDER_OVERRIDE\";"

# Redis is started (above, before the grid) by the script itself; this pane
# only follows its log. Ctrl+C here leaves Redis running.
CMD_DOCKER="$SETUP
cd \"\$NATIVE_DIR\";
docker compose logs -f redis
"

# The UR5, kinematically: IK against the transform tree, joints and link frames
# back into Redis. No ROS.
CMD_SIM="$SETUP
native_wait_for_redis;
cd \"\$SIM_DIR\";
native_cargo run --release --bin simple_robot_simulator
"

# Loads the scene and publishes /tf, /tf_static and /mesh_markers for RViz.
CMD_VIZ="$SETUP_ROS
native_wait_for_redis;
cd \"\$VIZ_DIR\";
native_cargo run --release
"

CMD_RVIZ="$SETUP_ROS
native_wait_for_redis;
rviz2 -d \"\$RVIZ_CONFIG\"
"

CMD_RUNNER="$SETUP
$ORDER_EXPORT
native_wait_for_redis;
cd \"\$DICE_CELL_DIR\";
native_cargo run --release --bin runner
"

CMD_CELL="$SETUP
native_wait_for_redis;
native_wait_for_runner;
cd \"\$DICE_CELL_DIR\";
native_cargo run --release --bin cell
"

CMD_GUI="$(gui_command)"

CMD_LINKS="$SETUP
clear; native_print_links
"

CMD_SHELL="$SETUP
clear
"

# --- The grid -----------------------------------------------------------------

tmux kill-session -t "$SESSION_NAME" 2>/dev/null
echo "Starting new tmux session '$SESSION_NAME'..."
tmux new-session -d -s "$SESSION_NAME" -n "DICE" -c "$SCRIPT_DIR"
tmux set-option -g history-limit 500000
tmux set-option -t "$SESSION_NAME:0" pane-border-status top
tmux set-option -t "$SESSION_NAME:0" pane-border-format ' #{pane_index} #{pane_title} '

PANE_COUNT=9
for (( pane_i = 1; pane_i < PANE_COUNT; pane_i++ )); do
    tmux split-window -t "$SESSION_NAME:0" -c "$SCRIPT_DIR"
    tmux select-layout -t "$SESSION_NAME:0" tiled >/dev/null
done

pane() {
    tmux select-pane -t "$SESSION_NAME:0.$1" -T "$2"
    [[ -n "$3" ]] && tmux send-keys -t "$SESSION_NAME:0.$1" "$3" C-m
}

pane 0 "Runner (model)"            "$CMD_RUNNER"
pane 1 "Cell (dice, gripper)"      "$CMD_CELL"
pane 2 "Robot simulator (UR5)"     "$CMD_SIM"
pane 3 "Redis Visualization"       "$CMD_VIZ"
pane 4 "RViz"                      "$CMD_RVIZ"
pane 5 "$GUI_PANE_TITLE"           "$CMD_GUI"
pane 6 "Redis"                     "$CMD_DOCKER"
pane 7 "Links"                     "$CMD_LINKS"
pane 8 "Shell"                     "$CMD_SHELL"

cat <<EOF

  micro_sp GUI          http://localhost:$MICRO_SP_GUI_PORT  (up once its pane has built it)
  RViz                  opens as a window on this desktop
  Place an order        GUI State tab: set \`order\` to e.g.  red:6 green:5 blue:4
  Disturb the cell      GUI State tab: set \`disturbance\` to  knock_tower | steal_top | bump red | reset
  Settings              $NATIVE_ENV_FILE  (./run_natively.sh --show)

EOF

echo "Attaching to session '$SESSION_NAME'. 'Ctrl+b d' detaches; './run_natively.sh --stop' stops it."
tmux attach-session -t "$SESSION_NAME"
