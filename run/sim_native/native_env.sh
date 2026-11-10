#!/bin/bash
#
# Shared environment for the native run.
#
# Sourced by install_natively.sh, by run_natively.sh, and by every tmux pane that
# run_natively.sh opens, so it is the one place that knows where things live.
# The values come from .env next to this file; anything relative in .env is
# resolved against the repository root, so a fresh clone works as it stands.
#
# Sourced, not executed: it must not call `exit`, and it has to be safe to
# source more than once.

NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$NATIVE_DIR/../.." && pwd)"
NATIVE_ENV_FILE="$NATIVE_DIR/.env"

# Absolute and ~-relative paths are taken as they are; everything else is
# relative to the repository root.
native_path() {
    case "$1" in
        "")   printf '' ;;
        /*)   printf '%s' "$1" ;;
        "~")  printf '%s' "$HOME" ;;
        "~/"*) printf '%s' "$HOME/${1#\~/}" ;;
        *)    printf '%s' "$REPO_ROOT/$1" ;;
    esac
}

# Plain KEY=value lines (compose reads the same file); `set -a` exports them,
# which is how they reach the crates.
if [[ -f "$NATIVE_ENV_FILE" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$NATIVE_ENV_FILE"
    set +a
fi

# --- ROS ---------------------------------------------------------------------
: "${ROS_DISTRO:=lyrical}"
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
: "${ROS_SETUP:=/opt/ros/$ROS_DISTRO/setup.bash}"
ROS_SETUP="$(native_path "$ROS_SETUP")"
export ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION

# --- Sources built from disk --------------------------------------------------
: "${DEPS_DIR:=native_deps}"
: "${VIZ_DIR:=$DEPS_DIR/r2r_redis_visualization}"
: "${SIM_DIR:=$DEPS_DIR/simple_robot_simulator}"
: "${UR_DRIVER_DIR:=$DEPS_DIR/ur_redis_driver}"
: "${UR_DESCRIPTION_DIR:=$UR_DRIVER_DIR/src/ur_description}"
: "${MICRO_SP_GUI_DIR:=$DEPS_DIR/micro_sp_gui}"
: "${DICE_CELL_DIR:=dice_cell}"
DEPS_DIR="$(native_path "$DEPS_DIR")"
VIZ_DIR="$(native_path "$VIZ_DIR")"
SIM_DIR="$(native_path "$SIM_DIR")"
UR_DRIVER_DIR="$(native_path "$UR_DRIVER_DIR")"
UR_DESCRIPTION_DIR="$(native_path "$UR_DESCRIPTION_DIR")"
MICRO_SP_GUI_DIR="$(native_path "$MICRO_SP_GUI_DIR")"
DICE_CELL_DIR="$(native_path "$DICE_CELL_DIR")"

: "${VIZ_GIT_URL:=https://github.com/sequenceplanner/r2r_redis_visualization.git}"
: "${VIZ_GIT_REF:=v0.4.0}"
: "${SIM_GIT_URL:=https://github.com/sequenceplanner/simple_robot_simulator.git}"
: "${SIM_GIT_REF:=master}"
: "${UR_REDIS_DRIVER_GIT_URL:=https://github.com/sequenceplanner/ur_redis_driver.git}"
: "${UR_REDIS_DRIVER_GIT_REF:=v0.3.1}"
: "${MICRO_SP_GUI_GIT_URL:=https://github.com/endre90/micro_sp_gui.git}"
: "${MICRO_SP_GUI_GIT_REF:=master}"
: "${TRUNK_VERSION:=v0.21.14}"

# --- Shared paths -------------------------------------------------------------
# The markers carry file:// URIs that RViz resolves itself; natively publisher
# and viewer share a filesystem, so that is automatic.
: "${MESHES_DIR:=shared_folder/meshes}"
: "${SCENARIO_DIR:=shared_folder/scenario}"
: "${RVIZ_CONFIG:=run/sim_docker/rviz/default.rviz}"
MESHES_DIR="$(native_path "$MESHES_DIR")"
SCENARIO_DIR="$(native_path "$SCENARIO_DIR")"
RVIZ_CONFIG="$(native_path "$RVIZ_CONFIG")"
export MESHES_DIR SCENARIO_DIR

# --- Redis --------------------------------------------------------------------
: "${REDIS_HOST:=127.0.0.1}"
: "${REDIS_PORT:=6379}"
export REDIS_HOST REDIS_PORT

# --- micro_sp GUI -------------------------------------------------------------
: "${MICRO_SP_GUI_PORT:=8080}"
: "${MICRO_SP_GUI_FRAMES_DIR:=shared_folder/scenario/exported}"
MICRO_SP_GUI_FRAMES_DIR="$(native_path "$MICRO_SP_GUI_FRAMES_DIR")"

# --- Robot --------------------------------------------------------------------
: "${ROBOT_ID:=r1}"
: "${ROBOT_MODEL:=ur5}"
: "${INITIAL_JOINT_STATE:=0,-1.5708,1.5708,-1.5708,-1.5708,0}"
export ROBOT_ID ROBOT_MODEL INITIAL_JOINT_STATE
# What simple_robot_simulator calls the description directory.
export DESCRIPTION_DIR="$UR_DESCRIPTION_DIR"

# --- micro_sp -----------------------------------------------------------------
: "${SP_INSTANCE_ID:=sp1}"
: "${RUST_LOG:=info}"
: "${MICRO_SP_ACTIVITY_LOG:=1}"
: "${MICRO_SP_ACTIVITY_LOG_DIR:=run/sim_native/logs/micro_sp_logs}"
MICRO_SP_ACTIVITY_LOG_DIR="$(native_path "$MICRO_SP_ACTIVITY_LOG_DIR")"
export SP_INSTANCE_ID RUST_LOG MICRO_SP_ACTIVITY_LOG MICRO_SP_ACTIVITY_LOG_DIR

# --- Fetching the git dependencies -------------------------------------------
#
# The crates declare their git dependencies as ssh://git@github.com/..., which
# cargo cannot fetch without a GitHub SSH key. The rewrite to https goes on
# cargo's own environment, one invocation at a time, and is deliberately not
# exported - an exported insteadOf would also turn a later `git push` to an
# ssh remote into https. Cargo's libgit2 fetcher ignores insteadOf, so it is
# also told to shell out to the git CLI.
native_https_rewrite() {
    if [[ "${GITHUB_HTTPS_REWRITE:-true}" == "false" ]]; then
        "$@"
        return
    fi
    CARGO_NET_GIT_FETCH_WITH_CLI=true \
        GIT_CONFIG_COUNT=1 \
        GIT_CONFIG_KEY_0="url.https://github.com/.insteadOf" \
        GIT_CONFIG_VALUE_0="ssh://git@github.com/" \
        "$@"
}

native_cargo() { native_https_rewrite cargo "$@"; }
native_trunk() { native_https_rewrite trunk "$@"; }

if ! command -v cargo >/dev/null 2>&1 && [[ -f "$HOME/.cargo/env" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/.cargo/env"
fi

# Sourcing a ROS setup file under `set -u` or `set -e` trips over ament's own
# shell code, so both are lifted for the duration.
native_source_ros() {
    local restore=""
    [[ $- == *u* ]] && restore+="u"
    [[ $- == *e* ]] && restore+="e"
    set +ue
    local ok=0
    if [[ -f "$ROS_SETUP" ]]; then
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
        ok=1
    else
        echo "ROS setup file not found: $ROS_SETUP" >&2
        echo "Install ROS 2 with ./install_natively.sh, or set ROS_SETUP in .env." >&2
    fi
    [[ "$restore" == *u* ]] && set -u
    [[ "$restore" == *e* ]] && set -e
    return $(( 1 - ok ))
}

# --- Startup gates ------------------------------------------------------------
#
# Compose orders the dockerized run; natively each pane waits for what it needs.
# A pane that is waiting says so on screen and nudges every NATIVE_WAIT_NUDGE
# seconds; nothing gives up on its own.

native_wait() {
    local label="$1" hint="$2"
    shift 2
    local waited=0 nudge="${NATIVE_WAIT_NUDGE:-30}"
    if [[ -n "${NATIVE_SKIP_GATES:-}" ]]; then
        echo "Not waiting for $label (--no-wait)."
        return 0
    fi
    if "$@"; then
        echo "$label is up."
        return 0
    fi
    echo "Waiting for $label ..."
    until "$@"; do
        sleep 1
        waited=$(( waited + 1 ))
        if (( nudge > 0 && waited % nudge == 0 )); then
            printf '  still waiting for %s (%ds)%s\n' "$label" "$waited" "${hint:+ - $hint}"
        fi
    done
    echo "$label is up!"
}

native_tcp_open() {
    (echo > "/dev/tcp/$1/$2") >/dev/null 2>&1
}

native_wait_for_redis() {
    native_wait "Redis on $REDIS_HOST:$REDIS_PORT" "it starts in the docker pane" \
        native_tcp_open "$REDIS_HOST" "$REDIS_PORT"
}

# The runner writes this once the model's state is in Redis. The cell gates on
# it: the cell's first camera reading has to land after the runner's seed.
native_wait_for_runner() {
    native_wait "the runner (model state pushed to Redis)" "see the Runner pane" \
        test -f /tmp/runner_ready.flag
}

native_print_links() {
    printf '\n'
    printf '  %-22s %s\n' \
        "http://localhost:$MICRO_SP_GUI_PORT"   "micro_sp GUI - write an order into \`order\` (State tab)" \
        "$REDIS_HOST:$REDIS_PORT"               "Redis" \
        "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"          "RViz: /tf /tf_static /mesh_markers"
    printf '\n'
}

native_print_env() {
    cat <<EOF

  Native run configuration ($NATIVE_ENV_FILE)

    Repository            $REPO_ROOT
    ROS setup             $ROS_SETUP
    Visualization         $VIZ_DIR
    Robot simulator       $SIM_DIR
    ur_description        $UR_DESCRIPTION_DIR
    dice_cell             $DICE_CELL_DIR
    micro_sp GUI          $MICRO_SP_GUI_DIR  (http://localhost:$MICRO_SP_GUI_PORT)
    Meshes                $MESHES_DIR
    Scene                 $SCENARIO_DIR
    RViz config           $RVIZ_CONFIG
    Activity logs         $MICRO_SP_ACTIVITY_LOG_DIR
    Redis                 $REDIS_HOST:$REDIS_PORT
    Robot                 $ROBOT_ID ($ROBOT_MODEL), speed ${ROBOT_SPEED:-30.0}
    Dice / pads           ${N_DICE:-6} / ${N_PADS:-3}, seed ${CELL_SEED:-<random>}
    Initial order         ${INITIAL_ORDER:-<none>}

EOF
}
