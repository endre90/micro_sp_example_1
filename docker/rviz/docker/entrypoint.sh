#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"

cleanup() {
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

DISPLAY_NUM="${DISPLAY#:}"

# `docker compose restart` reuses the container filesystem, so the X server's
# lock file and socket from the previous run are still there and it would refuse
# to start with "server is already active".
rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"

# Say out loud which rendering path we are on. Mesa silently falls back to
# llvmpipe when it cannot use a render node, and that fallback is the difference
# between ~30 fps and ~5 fps - so it must be visible in the logs rather than
# only showing up as bad performance.
if [ -n "$LIBGL_ALWAYS_SOFTWARE" ] && [ "$LIBGL_ALWAYS_SOFTWARE" != "0" ]; then
    echo "GPU: software rendering forced via LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE."
elif ls /dev/dri/renderD* >/dev/null 2>&1; then
    for node in /dev/dri/renderD*; do
        if [ -r "$node" ] && [ -w "$node" ]; then
            echo "GPU: render node $node is usable - expecting hardware rendering."
        else
            echo "GPU: render node $node present but NOT accessible to uid $(id -u)/gids $(id -G)." >&2
            echo "GPU: Mesa will silently fall back to llvmpipe. If this container was" >&2
            echo "GPU: changed to run as non-root, RENDER_GID must match the node's group." >&2
        fi
    done
else
    echo "GPU: no /dev/dri render node in this container - software rendering (llvmpipe)."
fi

echo "Starting Xvnc on $DISPLAY (${SCREEN_GEOMETRY}, depth ${SCREEN_DEPTH}) ..."
# -localhost so the raw VNC port is not reachable from outside the container;
# only the noVNC port is published. websockify below is the only client.
Xvnc "$DISPLAY" \
    -geometry "$SCREEN_GEOMETRY" \
    -depth "$SCREEN_DEPTH" \
    -SecurityTypes None \
    -AlwaysShared \
    -localhost yes \
    -rfbport "$VNC_PORT" \
    >/tmp/xvnc.log 2>&1 &

# Wait for the display to accept clients before anything tries to map a window.
# Without this check a dead X server turns into a confusing Qt plugin error from
# RViz several seconds later, rather than the actual problem.
for _ in $(seq 1 100); do
    xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
    sleep 0.2
done
if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    echo "Error: Xvnc never came up on $DISPLAY. Log follows:" >&2
    cat /tmp/xvnc.log >&2 || true
    exit 1
fi

# Report the renderer RViz will actually get, for the same reason as above.
if command -v glxinfo >/dev/null 2>&1; then
    echo "GPU: $(glxinfo -B 2>/dev/null | grep -i 'OpenGL renderer' || echo 'renderer unknown')"
fi

echo "Starting window manager ..."
matchbox-window-manager -use_titlebar no >/dev/null 2>&1 &

echo "Serving noVNC on port $NOVNC_PORT ..."
websockify --web /usr/share/novnc "$NOVNC_PORT" "localhost:$VNC_PORT" >/dev/null 2>&1 &

# An explicit command wins; otherwise run RViz with the scenario config.
if [ "$#" -gt 0 ]; then
    exec "$@"
fi

RVIZ_ARGS=()
if [ -r "$RVIZ_CONFIG" ]; then
    echo "Using RViz config $RVIZ_CONFIG"
    RVIZ_ARGS+=(-d "$RVIZ_CONFIG")
else
    echo "No readable RViz config at $RVIZ_CONFIG - starting with RViz defaults."
fi

echo "Open http://localhost:${NOVNC_HOST_PORT:-$NOVNC_PORT}/vnc.html?autoconnect=1&resize=remote"
exec rviz2 "${RVIZ_ARGS[@]}"
