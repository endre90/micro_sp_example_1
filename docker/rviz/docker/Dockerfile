# RViz2, rendered headlessly and served to the browser over noVNC.
#
# The native setup just runs `rviz2` against the host's X server. Clients are not
# guaranteed to be on a Linux host with a reachable X socket, so here RViz draws
# into a headless X server which is exported over VNC and served as a web page.
#
# That X server is TigerVNC's Xvnc rather than Xvfb, for two reasons:
#
#   1. Xvfb has no DRI3, so X11 clients under it can only ever get Mesa's
#      software rasterizer (llvmpipe) - even with a GPU passed into the
#      container. Xvnc is Xorg-derived and exposes DRI3 + GLX, so RViz gets the
#      real driver (e.g. radeonsi) when a render node is available.
#   2. Xvnc *is* the VNC encoder and tracks its own damage, so there is no
#      separate x11vnc process re-scraping the whole framebuffer every frame.
#
# Hardware rendering needs /dev/dri passed in - see run/sim_docker/docker-compose.gpu.yaml,
# which run_in_docker.sh layers on automatically when the host has a render node.
# Without it everything still works, just on llvmpipe.

ARG ROS_DISTRO=lyrical

FROM osrf/ros:${ROS_DISTRO}-desktop

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgl1-mesa-dri \
        # Qt 6.5+ wants this for its xcb platform plugin.
        libxcb-cursor0 \
        # A kiosk window manager: it fullscreens the single app, so RViz fills
        # the framebuffer whatever SCREEN_GEOMETRY is set to.
        matchbox-window-manager \
        # glxinfo, so the active GL renderer can be *verified* rather than assumed.
        mesa-utils \
        novnc \
        procps \
        tigervnc-standalone-server \
        websockify \
        x11-utils \
        # The desktop image ships the Qt SVG *libraries* but not the Qt image
        # format plugin, so every RViz toolbar icon fails to load without these.
        libqt5svg5 \
        qt6-svg-plugins && \
    rm -rf /var/lib/apt/lists/*

# RViz writes its window state here and logs an error if the directory is absent.
RUN mkdir -p /root/.rviz2

# noVNC ships its client as vnc.html; make / serve it so the URL is just the
# host and port.
RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

# Note: LIBGL_ALWAYS_SOFTWARE is deliberately NOT set. Mesa picks the hardware
# driver when a usable render node is present and falls back to llvmpipe when it
# is not, which is exactly the behaviour we want. Set it from .env to force
# software rendering.
ENV DISPLAY=:99 \
    SCREEN_GEOMETRY=1600x900 \
    SCREEN_DEPTH=24 \
    VNC_PORT=5900 \
    NOVNC_PORT=6080 \
    RVIZ_CONFIG=/opt/rviz/default.rviz \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1

EXPOSE 6080

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
