#!/bin/bash
#
# The dockerized twin of ../sim_native/run_natively.sh.
#
# Every component is a compose service here; this script builds the images,
# brings the stack up and lays a tmux grid over `docker compose logs -f` so the
# panes read like the native run. Plain `docker compose up` works just as well.
#
#   ./run_in_docker.sh                 build (cached) + up + attach
#   ./run_in_docker.sh --check         verify the prerequisites and exit
#   ./run_in_docker.sh --build         re-clone the upstream sources and rebuild
#   ./run_in_docker.sh --pull          as --build, and refresh the base images too
#   ./run_in_docker.sh --rebuild-gui   rebuild only the micro_sp GUI and restart it
#   ./run_in_docker.sh --no-build      skip the build step entirely
#   ./run_in_docker.sh --no-tmux       build + up, stream all logs to this terminal
#   ./run_in_docker.sh --order "..."   place this order once the runner is up
#   ./run_in_docker.sh --no-gpu        force software rendering for RViz
#   ./run_in_docker.sh --show          print the resolved settings and exit
#   ./run_in_docker.sh --stop          stop the stack
#   ./run_in_docker.sh --clean         stop the stack and delete its volumes
#   ./run_in_docker.sh --attach        attach to an already running grid
#
# The first run pulls the ROS and Rust base images and compiles five Rust
# projects: several minutes and a few GB. Later runs start in seconds.
#
# RViz renders on the GPU when the host exposes a DRM render node (detected
# below, wired up through docker-compose.gpu.yaml), and falls back to Mesa's
# software rasterizer otherwise.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Absolute, because --help reads the header back out of this file and the cd
# below would otherwise break a relative invocation.
SCRIPT_PATH="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

SESSION_NAME="dice_grid_docker"

DO_BUILD=1
FORCE_BUILD=0
PULL=0
USE_TMUX=1
USE_GPU=1
ACTION="up"
# Empty means "whatever .env says". --order overrides it for this run only.
ORDER_OVERRIDE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)    FORCE_BUILD=1 ;;
        --pull)     FORCE_BUILD=1; PULL=1 ;;
        --rebuild-gui) ACTION="rebuild-gui" ;;
        --no-build) DO_BUILD=0 ;;
        --no-tmux)  USE_TMUX=0 ;;
        --no-gpu)   USE_GPU=0 ;;
        # Exported below, where it beats .env in compose's interpolation.
        --order)    ORDER_OVERRIDE="${2:?--order needs a value, e.g. red:6 green:5}"; shift ;;
        --order=*)  ORDER_OVERRIDE="${1#*=}" ;;
        --show)     ACTION="show" ;;
        --check)    ACTION="check" ;;
        --stop)     ACTION="stop" ;;
        --clean)    ACTION="clean" ;;
        --attach)   ACTION="attach" ;;
        -h|--help)  sed -n '2,28p' "$SCRIPT_PATH"; exit 0 ;;
        *)          echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

if ! docker compose version >/dev/null 2>&1; then
    echo "Error: 'docker compose' is not available. Install Docker Engine with the Compose plugin." >&2
    exit 1
fi

# --- Reading .env -------------------------------------------------------------
#
# Deliberately not `source`: .env holds values that would shadow compose's own
# interpolation if they landed in this shell. env_get reads one key out of it
# without sourcing the file.

ENV_FILE="$SCRIPT_DIR/.env"

env_get() {
    local key="$1" default="$2" value=""
    if [[ -f "$ENV_FILE" ]]; then
        value="$(sed -n "s/^[[:space:]]*${key}=//p" "$ENV_FILE" | tail -n1)"
    fi
    value="${value%%#*}"
    value="${value//[[:space:]]/}"
    value="${value%\"}"
    value="${value#\"}"
    [[ -n "$value" ]] && printf '%s' "$value" || printf '%s' "$default"
}

show_settings() {
    cat <<EOF

  Dockerized run configuration ($ENV_FILE)

    Robot                 $(env_get ROBOT_ID r1) ($(env_get ROBOT_MODEL ur5)), speed $(env_get ROBOT_SPEED 30.0)
    Dice / pads           $(env_get N_DICE 6) / $(env_get N_PADS 3), seed $(env_get CELL_SEED "<random>")
    Initial order         ${ORDER_OVERRIDE:-$(env_get INITIAL_ORDER "<none>")}
    micro_sp instance     $(env_get SP_INSTANCE_ID sp1)
    Scene                 ../../shared_folder/scenario
    Meshes                ../../shared_folder/meshes

    Upstream refs         viz $(env_get VIZ_GIT_REF v0.4.0) | simulator $(env_get SIM_GIT_REF master) | ur_description $(env_get UR_REDIS_DRIVER_GIT_REF v0.3.1) | gui $(env_get MICRO_SP_GUI_GIT_REF master)

    Redis                 localhost:$(env_get REDIS_HOST_PORT 6379)
    micro_sp GUI          http://localhost:$(env_get MICRO_SP_GUI_HOST_PORT 8080)
    RViz                  http://localhost:$(env_get RVIZ_HOST_PORT 6081)/vnc.html?autoconnect=1&resize=remote

EOF
}

# Everything the dockerized run needs, which is Docker plus this repository.
check_prerequisites() {
    local missing=0
    if ! docker compose version >/dev/null 2>&1; then
        echo "  missing  docker compose" >&2; missing=1
    elif ! docker info >/dev/null 2>&1; then
        echo "  missing  a running docker daemon (or your user is not in the docker group)" >&2
        missing=1
    fi
    local ctx
    for ctx in dice_cell simple_robot_simulator r2r_redis_visualization rviz micro_sp_gui; do
        [[ -d "$SCRIPT_DIR/../../docker/$ctx/docker" ]] ||
            { echo "  missing  docker/$ctx/docker" >&2; missing=1; }
    done
    local path
    for path in shared_folder/meshes shared_folder/scenario; do
        [[ -d "$SCRIPT_DIR/../../$path" ]] || { echo "  missing  $path" >&2; missing=1; }
    done
    # The native stack publishes the same ports; both up at once is a port clash.
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q '_dice_native$'; then
        echo "  warning  the native stack is up - stop it first (../sim_native/run_natively.sh --stop)" >&2
    fi
    return $missing
}

# Pick the render node to hand RViz. An open-driver GPU (amdgpu/i915/radeon) is
# preferred: Mesa drives it with no host packages to install, unlike NVIDIA which
# would need the container toolkit. Falls back to the first render node found.
detect_render_node() {
    local path card driver first=""
    for path in /dev/dri/by-path/*-render; do
        [[ -e "$path" ]] || continue
        card="${path%-render}-card"
        [[ -e "$card" ]] || continue
        driver="$(basename "$(readlink -f "/sys/class/drm/$(basename "$(readlink -f "$card")")/device/driver" 2>/dev/null)" 2>/dev/null || true)"
        [[ -n "$first" ]] || first="$(readlink -f "$path")"
        case "$driver" in
            amdgpu|i915|xe|radeon)
                readlink -f "$path"
                return 0
                ;;
        esac
    done
    if [[ -n "$first" ]]; then
        echo "$first"
        return 0
    fi
    # No by-path entries (unusual, but possible in VMs) - take any render node.
    for path in /dev/dri/renderD*; do
        [[ -e "$path" ]] && { echo "$path"; return 0; }
    done
    return 1
}

# A shell variable beats the .env file for compose interpolation, so exporting it
# here is all --model needs.
if [[ -n "${ORDER_OVERRIDE:-}" ]]; then
    export INITIAL_ORDER="$ORDER_OVERRIDE"
fi

DC=(docker compose)
GPU_STATUS="software rendering (no /dev/dri render node on this host)"

if [[ "$USE_GPU" == "0" ]]; then
    GPU_STATUS="software rendering (forced with --no-gpu)"
    export LIBGL_ALWAYS_SOFTWARE=1
elif DRI_RENDER_NODE="$(detect_render_node 2>/dev/null)" && [[ -n "${DRI_RENDER_NODE:-}" ]]; then
    # The device's group GID varies between hosts, so read it off the node
    # itself; a wrong GID leaves the device unreadable and Mesa drops silently
    # back to llvmpipe.
    RENDER_GID="$(stat -c %g "$DRI_RENDER_NODE")"
    export DRI_RENDER_NODE RENDER_GID
    DC+=(-f docker-compose.yaml -f docker-compose.gpu.yaml)
    GPU_STATUS="hardware rendering via $DRI_RENDER_NODE (group gid $RENDER_GID)"
fi

case "$ACTION" in
    stop)
        echo "Stopping the stack..."
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        "${DC[@]}" down
        exit 0
        ;;
    clean)
        echo "Stopping the stack and removing its volumes..."
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        "${DC[@]}" down -v
        exit 0
        ;;
    attach)
        exec tmux attach-session -t "$SESSION_NAME"
        ;;
    show)
        show_settings
        exit 0
        ;;
    check)
        echo "Checking the dockerized prerequisites..."
        if check_prerequisites; then
            echo "  everything the dockerized run needs is in place."
            exit 0
        fi
        exit 1
        ;;
    rebuild-gui)
        # The bust goes to this one service rather than to the whole build, so viz
        # and the simulator are not re-cloned and recompiled for a commit that is not
        # theirs. .env is deliberately not sourced into this shell (see the comment
        # over env_get), so the ref and the port are read out of it.
        echo "Rebuilding micro_sp_gui from $(env_get MICRO_SP_GUI_GIT_REF master)"
        echo "(wasm frontend + server, from scratch - a few minutes)..."
        if ! "${DC[@]}" build --build-arg "CACHE_BUST=$(date +%s)" micro_sp_gui; then
            echo "Build failed - the running GUI is untouched." >&2
            exit 1
        fi
        # Only this service, and no --remove-orphans: with a service named it has
        # been known to reap the containers that were not, which here would be the
        # rest of the stack.
        echo "Recreating the GUI container..."
        "${DC[@]}" up -d micro_sp_gui
        printf '\n  micro_sp GUI    http://localhost:%s\n\n' "$(env_get MICRO_SP_GUI_HOST_PORT 8080)"
        exit 0
        ;;
esac

if ! check_prerequisites; then
    echo >&2
    echo "Fix the above, then re-run. ./run_in_docker.sh --check re-checks." >&2
    exit 1
fi

mkdir -p logs

if [[ "$DO_BUILD" == "1" ]]; then
    BUILD_ARGS=()
    if [[ "$FORCE_BUILD" == "1" ]]; then
        # viz, the simulator and the GUI clone from git, and a clone layer looks unchanged to
        # Docker even when the upstream branch has moved. Bumping CACHE_BUST
        # re-clones and recompiles those without throwing away the (expensive)
        # apt/ROS/toolchain layers underneath.
        BUILD_ARGS+=(--build-arg "CACHE_BUST=$(date +%s)")
        echo "Building images (re-cloning upstream sources)..."
    else
        echo "Building images (first run pulls ROS + Rust and takes a while)..."
    fi
    if [[ "$PULL" == "1" ]]; then
        BUILD_ARGS+=(--pull)
    fi
    "${DC[@]}" build "${BUILD_ARGS[@]}" || {
        echo "Build failed." >&2
        exit 1
    }
fi

echo "Starting the stack..."
echo "RViz: $GPU_STATUS"
# --force-recreate: containers left over from before Docker last restarted (e.g.
# a reboot) can start with no network at all - loopback only, no published
# ports, no DNS between services - and the stack then waits on itself forever.
"${DC[@]}" up -d --force-recreate --remove-orphans

RVIZ_PORT="$(env_get RVIZ_HOST_PORT 6081)"
GUI_PORT="$(env_get MICRO_SP_GUI_HOST_PORT 8080)"

print_urls() {
    cat <<EOF

  RViz2 (noVNC)   http://localhost:${RVIZ_PORT}/vnc.html?autoconnect=1&resize=remote
  micro_sp GUI    http://localhost:${GUI_PORT}

  Place an order  GUI State tab: set \`order\` to e.g.  red:6 green:5 blue:4
  Disturb         GUI State tab: set \`disturbance\` to  knock_tower | steal_top | bump red | reset
  RViz rendering  ${GPU_STATUS}
  Settings        ${ENV_FILE}  (./run_in_docker.sh --show)

EOF
}

if [[ "$USE_TMUX" == "0" ]]; then
    print_urls
    exec "${DC[@]}" logs -f
fi

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux not found - streaming all logs instead."
    print_urls
    exec "${DC[@]}" logs -f
fi

tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
echo "Starting new tmux session '$SESSION_NAME'..."

tmux new-session -d -s "$SESSION_NAME" -n "DOCKER" -c "$SCRIPT_DIR"

tmux set-option -g history-limit 500000

tmux set-option -t "$SESSION_NAME:0" pane-border-status top
# Single-quoted: #{...} is a tmux format string and must reach tmux unexpanded.
tmux set-option -t "$SESSION_NAME:0" pane-border-format ' #{pane_index} #{pane_title} '

# Same 3x3 grid as the native run.

tmux split-window -t "$SESSION_NAME:0.0" -v -p 66 -c "$SCRIPT_DIR"
tmux split-window -t "$SESSION_NAME:0.1" -v -p 50 -c "$SCRIPT_DIR"

tmux split-window -t "$SESSION_NAME:0.0" -h -p 66 -c "$SCRIPT_DIR"
tmux split-window -t "$SESSION_NAME:0.3" -h -p 50 -c "$SCRIPT_DIR"

tmux split-window -t "$SESSION_NAME:0.1" -h -p 66 -c "$SCRIPT_DIR"
tmux split-window -t "$SESSION_NAME:0.5" -h -p 50 -c "$SCRIPT_DIR"

tmux split-window -t "$SESSION_NAME:0.2" -h -p 66 -c "$SCRIPT_DIR"
tmux split-window -t "$SESSION_NAME:0.7" -h -p 50 -c "$SCRIPT_DIR"

tmux select-layout -t "$SESSION_NAME:0" tiled

# pane index -> "title:service" (empty service = plain shell)
pane() {
    local index="$1" title="$2" service="$3"
    tmux select-pane -t "$SESSION_NAME:0.$index" -T "$title"
    if [[ -n "$service" ]]; then
        tmux send-keys -t "$SESSION_NAME:0.$index" \
            "docker compose logs -f --tail=200 $service" C-m
    fi
}

pane 0 "Runner (model)"        "runner"
pane 1 "Cell (dice, gripper)"  "cell"
pane 2 "Robot simulator (UR5)" "robot"
pane 3 "Redis Visualization"   "viz"
pane 4 "RViz (noVNC)"          "rviz"
pane 5 "micro_sp GUI"          "micro_sp_gui"
pane 6 "Redis"                 "redis init_flags"
pane 7 "Stack status"          ""

tmux send-keys -t "$SESSION_NAME:0.7" \
    "watch -n 2 'docker compose ps --format \"table {{.Service}}\t{{.Status}}\"'" C-m

pane 8 "Shell"                 ""
tmux send-keys -t "$SESSION_NAME:0.8" "clear; docker compose ps" C-m

print_urls

echo "Attaching to session '$SESSION_NAME'."
echo "Use 'Ctrl+b' then 'Arrow Keys' to move between panes."
echo "Detaching with 'Ctrl+b d' leaves the stack running; './run_in_docker.sh --stop' stops it."
tmux attach-session -t "$SESSION_NAME"
