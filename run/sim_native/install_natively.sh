#!/bin/bash
#
# Sets up everything the native run needs, so that ./run_natively.sh works on a
# machine that starts with nothing but this repository:
#
#   1. the system packages the Rust crates and ROS need to compile
#   2. ROS 2 (the distro named in .env) including RViz
#   3. the Rust toolchain, via rustup, with the wasm target and trunk for the GUI
#   4. Docker with the Compose plugin - Redis still runs in a container
#   5. the upstream repositories: the visualization bridge, the robot
#      simulator, the micro_sp GUI, and ur_redis_driver for the UR5's description
#   6. a release build of dice_cell, the simulator, the bridge and the GUI
#
# It checks what is already there and skips it, so it is also the "update
# everything" script - re-running it fetches the pinned refs and rebuilds.
#
#   ./install_natively.sh              install or update everything that is missing
#   ./install_natively.sh --check      report what is missing and change nothing
#   ./install_natively.sh --deps       system packages, ROS, Rust and Docker only
#   ./install_natively.sh --sources    clone or update the upstream sources only
#   ./install_natively.sh --build      compile the Rust projects only
#   ./install_natively.sh --no-docker  leave Docker alone
#   ./install_natively.sh --no-build   fetch everything but do not compile
#   ./install_natively.sh -y           do not ask anything, assume yes
#
# The apt parts are Ubuntu/Debian only; on anything else it prints what is needed
# and stops. Which ROS distro, which git refs and where the sources go are all
# read from .env.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"

# shellcheck source=native_env.sh
source "$SCRIPT_DIR/native_env.sh"

DO_DEPS=1
DO_SOURCES=1
DO_BUILD=1
WITH_DOCKER=1
CHECK_ONLY=0
ASSUME_YES=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check)     CHECK_ONLY=1 ;;
        --deps)      DO_DEPS=1; DO_SOURCES=0; DO_BUILD=0 ;;
        --sources)   DO_DEPS=0; DO_SOURCES=1; DO_BUILD=0 ;;
        --build)     DO_DEPS=0; DO_SOURCES=0; DO_BUILD=1 ;;
        --no-docker) WITH_DOCKER=0 ;;
        --no-build)  DO_BUILD=0 ;;
        -y|--yes)    ASSUME_YES=1 ;;
        -h|--help)   sed -n '2,29p' "$SCRIPT_PATH" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)           echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# --- Output -------------------------------------------------------------------

step()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
info()  { printf '    %s\n' "$*"; }
ok()    { printf '    \033[32mok\033[0m       %s\n' "$*"; }
miss()  { printf '    \033[33mmissing\033[0m  %s\n' "$*"; }
warn()  { printf '    \033[33mwarning\033[0m  %s\n' "$*" >&2; }
die()   { printf '\n\033[31mError:\033[0m %s\n' "$*" >&2; exit 1; }

confirm() {
    if [[ "$ASSUME_YES" == "1" ]]; then return 0; fi
    if [[ ! -t 0 ]]; then return 0; fi
    local reply
    read -r -p "    $1 [Y/n] " reply
    [[ -z "$reply" || "$reply" =~ ^[Yy] ]]
}

# --- Privileges and package manager -------------------------------------------

SUDO=""
if [[ "$(id -u)" != "0" ]]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    fi
fi

HAS_APT=0
command -v apt-get >/dev/null 2>&1 && HAS_APT=1

OS_ID=""
OS_CODENAME=""
if [[ -r /etc/os-release ]]; then
    OS_ID="$(. /etc/os-release && echo "${ID:-}")"
    OS_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME:-}")"
fi

APT_UPDATED=0
apt_update_once() {
    if [[ "$APT_UPDATED" == "0" ]]; then
        $SUDO apt-get update
        APT_UPDATED=1
    fi
}

require_apt() {
    if [[ "$HAS_APT" != "1" ]]; then
        die "$1 has to be installed by hand on this system - apt-get was not found."
    fi
    if [[ -z "$SUDO" && "$(id -u)" != "0" ]]; then
        die "installing $1 needs root, and sudo was not found."
    fi
}

# --- 1. System packages -------------------------------------------------------
#
# The same set the docker builders install, plus tmux for the pane grid and RViz's
# Qt xcb dependency. clang/libclang-dev are what r2r's binding generation needs.

APT_PACKAGES=(
    build-essential
    ca-certificates
    clang
    cmake
    curl
    git
    gnupg
    libclang-dev
    libssl-dev
    libxcb-cursor0
    pkg-config
    python3
    tmux
)

missing_apt_packages() {
    local pkg missing=()
    for pkg in "${APT_PACKAGES[@]}"; do
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            missing+=("$pkg")
        fi
    done
    printf '%s\n' "${missing[@]:-}"
}

install_system_packages() {
    step "System packages"
    if [[ "$HAS_APT" != "1" ]]; then
        warn "no apt-get on this system - install the equivalents of: ${APT_PACKAGES[*]}"
        return 0
    fi

    local missing
    mapfile -t missing < <(missing_apt_packages)
    # mapfile keeps one empty element when nothing is missing.
    if [[ "${#missing[@]}" == "0" || -z "${missing[0]}" ]]; then
        ok "all ${#APT_PACKAGES[@]} build dependencies present"
        return 0
    fi

    miss "${missing[*]}"
    if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
    require_apt "the build dependencies"
    confirm "Install them with apt?" || return 0
    apt_update_once
    $SUDO apt-get install -y --no-install-recommends "${missing[@]}"
    ok "build dependencies installed"
}

# --- 2. ROS 2 -----------------------------------------------------------------

: "${ROS_PACKAGES:=ros-$ROS_DISTRO-desktop}"

ros_apt_repo_present() {
    grep -rqs "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null
}

# The documented way in since Jazzy: a small .deb that drops in the repository
# definition and its signing key, instead of hand-managing a keyring file.
add_ros_apt_repo() {
    local release deb
    info "Adding the ROS 2 apt repository..."
    release="$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        grep -F '"tag_name"' | head -n1 | cut -d'"' -f4)"
    [[ -n "$release" ]] || die "could not determine the latest ros-apt-source release. Add the repository by hand: https://docs.ros.org/en/$ROS_DISTRO/Installation.html"

    deb="$(mktemp --suffix=.deb)"
    curl -fsSL -o "$deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.${OS_CODENAME}_all.deb" ||
        die "no ros-apt-source build for '$OS_CODENAME'. Add the repository by hand: https://docs.ros.org/en/$ROS_DISTRO/Installation.html"

    $SUDO apt-get install -y "$deb"
    rm -f "$deb"
    APT_UPDATED=0
    apt_update_once
}

install_ros() {
    step "ROS 2 ($ROS_DISTRO)"
    if [[ -f "$ROS_SETUP" ]]; then
        ok "$ROS_SETUP"
        if [[ "$ROS_PACKAGES" == *desktop* ]] && ! command -v rviz2 >/dev/null 2>&1 &&
           [[ ! -x "/opt/ros/$ROS_DISTRO/bin/rviz2" ]]; then
            warn "ROS is installed but rviz2 was not found - install ros-$ROS_DISTRO-rviz2"
        fi
        return 0
    fi

    miss "$ROS_SETUP"
    if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
    require_apt "ROS 2"
    confirm "Install $ROS_PACKAGES with apt? (a few GB)" || return 0

    ros_apt_repo_present || add_ros_apt_repo
    apt_update_once

    if ! apt-cache show "$ROS_PACKAGES" >/dev/null 2>&1; then
        die "$ROS_PACKAGES is not available for ${OS_ID:-this system} ${OS_CODENAME:-}.
       ROS $ROS_DISTRO may not be released for this release - check
       https://docs.ros.org/en/$ROS_DISTRO/Installation.html and set ROS_DISTRO in .env
       to a distro that is (the docker run in ../sim_docker has no such constraint)."
    fi

    $SUDO apt-get install -y "$ROS_PACKAGES"
    [[ -f "$ROS_SETUP" ]] || die "installed $ROS_PACKAGES but $ROS_SETUP is still missing."
    ok "ROS 2 $ROS_DISTRO installed"
}

# --- 3. Rust ------------------------------------------------------------------

install_rust() {
    step "Rust toolchain"
    if command -v cargo >/dev/null 2>&1; then
        ok "$(cargo --version)"
        install_gui_toolchain
        return 0
    fi

    miss "cargo"
    if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
    confirm "Install the stable Rust toolchain with rustup?" || return 0

    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs |
        sh -s -- -y --default-toolchain stable --profile default --no-modify-path
    # rustup only puts cargo on PATH for new shells, so pull it into this one.
    # shellcheck disable=SC1091
    source "$HOME/.cargo/env"
    ok "$(cargo --version)"
    info "Add ~/.cargo/bin to your PATH in this shell with: source \$HOME/.cargo/env"
    install_gui_toolchain
}

# The GUI's frontend is egui compiled to wasm, built by trunk. The prebuilt trunk
# binary rather than `cargo install trunk`, which would compile it from source,
# and the version the GUI's Docker image pins.
install_gui_toolchain() {
    if rustup target list --installed 2>/dev/null | grep -qx wasm32-unknown-unknown; then
        ok "wasm32-unknown-unknown target"
    else
        miss "the wasm32-unknown-unknown target"
        if [[ "$CHECK_ONLY" != "1" ]]; then
            command -v rustup >/dev/null 2>&1 ||
                die "rustup is needed to add the wasm target - this cargo is not managed by rustup."
            rustup target add wasm32-unknown-unknown
            ok "wasm32-unknown-unknown target added"
        fi
    fi

    if command -v trunk >/dev/null 2>&1; then
        ok "$(trunk --version)"
        return 0
    fi
    miss "trunk"
    if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
    confirm "Install trunk $TRUNK_VERSION into ~/.cargo/bin?" || return 0

    local target
    case "$(uname -m)" in
        x86_64)        target=x86_64-unknown-linux-gnu ;;
        aarch64|arm64) target=aarch64-unknown-linux-gnu ;;
        *) die "no prebuilt trunk for $(uname -m) - install it with: cargo install --locked trunk" ;;
    esac
    mkdir -p "$HOME/.cargo/bin"
    curl -fsSL "https://github.com/trunk-rs/trunk/releases/download/$TRUNK_VERSION/trunk-$target.tar.gz" |
        tar -xz -C "$HOME/.cargo/bin" trunk
    ok "$("$HOME/.cargo/bin/trunk" --version)"
}

# --- 4. Docker ----------------------------------------------------------------

install_docker() {
    step "Docker"
    if [[ "$WITH_DOCKER" == "0" ]]; then
        info "skipped (--no-docker)"
        return 0
    fi

    if docker compose version >/dev/null 2>&1; then
        ok "$(docker compose version | head -n1)"
    else
        miss "docker compose"
        if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
        require_apt "Docker"
        confirm "Install Docker Engine and the Compose plugin from Docker's apt repository?" || return 0

        local repo="https://download.docker.com/linux/${OS_ID:-ubuntu}"
        $SUDO install -m 0755 -d /etc/apt/keyrings
        curl -fsSL "$repo/gpg" | $SUDO gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        $SUDO chmod a+r /etc/apt/keyrings/docker.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] $repo ${OS_CODENAME} stable" |
            $SUDO tee /etc/apt/sources.list.d/docker.list >/dev/null
        APT_UPDATED=0
        apt_update_once
        $SUDO apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        ok "$(docker compose version | head -n1)"
    fi

    # A Docker that only answers to root turns into a permission error halfway
    # through the run, which is a confusing place to find out about it.
    if ! docker info >/dev/null 2>&1; then
        miss "the Docker daemon is not reachable as $(id -un)"
        if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
        if ! id -nG "$(id -un)" | tr ' ' '\n' | grep -qx docker; then
            if confirm "Add $(id -un) to the 'docker' group?"; then
                $SUDO groupadd -f docker
                $SUDO usermod -aG docker "$(id -un)"
                warn "log out and back in (or run 'newgrp docker') for the new group to apply"
            fi
        else
            warn "you are in the 'docker' group but the daemon is not answering - is it running? ($SUDO systemctl start docker)"
        fi
    fi
}

# --- 5. Upstream sources ------------------------------------------------------
#
# The dockerized run clones these at image build time; natively they are checkouts
# on disk that you can edit, which is the point of running natively at all.

clone_or_update() {
    local name="$1" dir="$2" url="$3" ref="$4"

    if [[ ! -d "$dir/.git" ]]; then
        if [[ -d "$dir" && -n "$(ls -A "$dir" 2>/dev/null)" ]]; then
            warn "$name: $dir exists but is not a git checkout - leaving it alone"
            return 0
        fi
        miss "$name ($dir)"
        if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi
        info "Cloning $url ..."
        git clone "$url" "$dir"
        git -C "$dir" checkout "$ref"
        ok "$name at $(git -C "$dir" rev-parse --short HEAD)"
        return 0
    fi

    ok "$name ($dir at $(git -C "$dir" rev-parse --short HEAD))"
    if [[ "$CHECK_ONLY" == "1" ]]; then return 0; fi

    # Never touch a checkout somebody is working in - this script is also run to
    # update, and that must not silently move a developer off their own changes.
    if [[ -n "$(git -C "$dir" status --porcelain)" ]]; then
        info "$name has local changes - not fetching or checking out"
        return 0
    fi

    git -C "$dir" fetch --tags --prune origin
    git -C "$dir" checkout "$ref"
    # A branch needs fast-forwarding as well; a tag or commit is already there.
    if git -C "$dir" symbolic-ref -q HEAD >/dev/null; then
        git -C "$dir" merge --ff-only "origin/$ref" >/dev/null 2>&1 ||
            warn "$name: could not fast-forward to origin/$ref"
    fi
    info "$name now at $(git -C "$dir" rev-parse --short HEAD)"
}

install_sources() {
    step "Upstream sources"
    if ! command -v git >/dev/null 2>&1; then
        miss "git"
        return 0
    fi
    [[ "$CHECK_ONLY" == "1" ]] || mkdir -p "$DEPS_DIR"
    clone_or_update "r2r_redis_visualization" "$VIZ_DIR" "$VIZ_GIT_URL" "$VIZ_GIT_REF"
    clone_or_update "simple_robot_simulator" "$SIM_DIR" "$SIM_GIT_URL" "$SIM_GIT_REF"
    clone_or_update "ur_redis_driver (for ur_description)" "$UR_DRIVER_DIR" "$UR_REDIS_DRIVER_GIT_URL" "$UR_REDIS_DRIVER_GIT_REF"
    clone_or_update "micro_sp_gui" "$MICRO_SP_GUI_DIR" "$MICRO_SP_GUI_GIT_URL" "$MICRO_SP_GUI_GIT_REF"

    if [[ -f "$UR_DESCRIPTION_DIR/urdf/$ROBOT_MODEL.urdf" ]]; then
        ok "ur_description ($UR_DESCRIPTION_DIR, $ROBOT_MODEL)"
    else
        miss "the $ROBOT_MODEL URDF - expected at $UR_DESCRIPTION_DIR/urdf (set UR_DESCRIPTION_DIR in .env)"
    fi
}

# --- 6. Build -----------------------------------------------------------------

build_crate() {
    local name="$1" dir="$2" needs_ros="$3"
    shift 3

    if [[ ! -f "$dir/Cargo.toml" ]]; then
        miss "$name: no Cargo.toml in $dir - run ./install_natively.sh --sources first"
        return 0
    fi
    if [[ "$CHECK_ONLY" == "1" ]]; then
        # A release binary that is newer than nothing is still a build worth
        # reporting; cargo decides whether it is up to date when it actually runs.
        if compgen -G "$dir/target/release/*" >/dev/null; then
            ok "$name has been built ($dir/target/release)"
        else
            miss "$name has not been built yet"
        fi
        return 0
    fi

    info "Building $name ..."
    (
        # r2r generates its Rust message types from the *sourced* ROS
        # environment, so the visualization bridge has to be built with it.
        if [[ "$needs_ros" == "ros" ]]; then
            native_source_ros || die "cannot build $name without ROS"
        fi
        cd "$dir"
        native_cargo build --release "$@"
    )
    ok "$name built"
}

install_build() {
    step "Building the Rust projects (first time takes a while)"
    if ! command -v cargo >/dev/null 2>&1; then
        miss "cargo - install the Rust toolchain first"
        return 0
    fi
    build_crate "dice_cell" "$DICE_CELL_DIR" "" --bins
    build_crate "simple_robot_simulator" "$SIM_DIR" "" --bin simple_robot_simulator
    build_crate "r2r_redis_visualization" "$VIZ_DIR" "ros"
    build_gui
}

# Two builds: trunk for the wasm frontend into the checkout's dist/, and the
# server that serves it. Its pane does both again on every start, which is cheap
# once this has done them the first time.
build_gui() {
    build_crate "micro_sp_gui server" "$MICRO_SP_GUI_DIR" "" -p micro_sp_gui_server
    [[ "$CHECK_ONLY" == "1" || ! -f "$MICRO_SP_GUI_DIR/Cargo.toml" ]] && return 0
    if ! command -v trunk >/dev/null 2>&1; then
        miss "trunk - the GUI frontend is not built (./install_natively.sh --deps)"
        return 0
    fi
    info "Building the micro_sp_gui frontend ..."
    ( cd "$MICRO_SP_GUI_DIR" && native_trunk build --release )
    ok "micro_sp_gui frontend built ($MICRO_SP_GUI_DIR/dist)"
}

# --- Run ----------------------------------------------------------------------

if [[ "$CHECK_ONLY" == "1" ]]; then
    step "Checking the native prerequisites (nothing will be installed)"
fi

if [[ "$DO_DEPS" == "1" ]]; then
    install_system_packages
    install_ros
    install_rust
    install_docker
fi

if [[ "$DO_SOURCES" == "1" ]]; then install_sources; fi
if [[ "$DO_BUILD" == "1" ]]; then install_build; fi

mkdir -p "$MICRO_SP_ACTIVITY_LOG_DIR" 2>/dev/null || true

if [[ "$CHECK_ONLY" == "1" ]]; then
    printf '\nRun ./install_natively.sh to install whatever is marked missing.\n'
    exit 0
fi

step "Done"
native_print_env
cat <<EOF
  Start the stack with:

      cd $SCRIPT_DIR
      ./run_natively.sh

EOF
