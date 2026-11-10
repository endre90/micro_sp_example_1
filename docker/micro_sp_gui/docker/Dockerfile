# The micro_sp web GUI: https://github.com/endre90/micro_sp_gui
#
# Two builds in one image: the frontend is egui compiled to wasm32 (wasm has no
# TCP sockets, so the browser cannot speak Redis itself), and the server is an
# axum binary that holds the one ConnectionManager, polls Redis and serves that
# frontend. The runtime stage therefore carries a binary *and* a `dist/`.
#
# The repository is a workspace (`crates/{protocol,server,web}`): the binary
# `micro_sp_gui` comes from the package `micro_sp_gui_server`, which is why the
# two names differ below. `trunk build` is run at the root, where Trunk.toml
# points it at index.html and puts the bundle in ./dist.

FROM rust:1.97-bookworm AS builder

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git curl pkg-config libssl-dev ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# `micro_sp` and `ur_redis_driver` are declared with `ssh://git@github.com/...`
# URLs but are public, so rewrite them to https rather than requiring the builder
# to hold a GitHub SSH key. Cargo's built-in libgit2 fetcher does not honour
# `insteadOf`, so it has to be told to shell out to the git CLI, which does.
# Both rewrites target the same config key, so the second one needs --add or it
# would overwrite the first instead of adding to it.
RUN git config --global --add url."https://github.com/".insteadOf "ssh://git@github.com/" && \
    git config --global --add url."https://github.com/".insteadOf "git@github.com:"
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true

# Trunk builds the wasm frontend. The prebuilt binary rather than
# `cargo install trunk`, which would compile it from source for no benefit; the
# version is pinned for the same reason the git refs below are.
ARG TRUNK_VERSION=v0.21.14
RUN arch="$(dpkg --print-architecture)" && \
    case "$arch" in \
        amd64) target=x86_64-unknown-linux-gnu ;; \
        arm64) target=aarch64-unknown-linux-gnu ;; \
        *)     echo "unsupported architecture: $arch" >&2; exit 1 ;; \
    esac && \
    curl -fsSL "https://github.com/trunk-rs/trunk/releases/download/$TRUNK_VERSION/trunk-$target.tar.gz" \
        | tar -xz -C /usr/local/bin trunk
RUN rustup target add wasm32-unknown-unknown

ARG MICRO_SP_GUI_GIT_URL=https://github.com/endre90/micro_sp_gui.git
ARG MICRO_SP_GUI_GIT_REF=master

# Bump to force a re-clone when the ref string has not changed but what it
# points at has: --build-arg CACHE_BUST=$(date +%s). The default ref is a
# branch, so this is the normal way to pick up a push - both launchers pass it
# for --build and --rebuild-gui. It would only be unnecessary on a release tag.
ARG CACHE_BUST=0

RUN git clone "$MICRO_SP_GUI_GIT_URL" /src && \
    cd /src && \
    git checkout "$MICRO_SP_GUI_GIT_REF"

WORKDIR /src
# The frontend first: it is the cheaper of the two, so a failure there is not paid
# for with a full server build.
RUN trunk build --release
RUN cargo build --release -p micro_sp_gui_server

FROM debian:bookworm-slim

RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates libssl3 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /src/target/release/micro_sp_gui /usr/local/bin/micro_sp_gui
COPY --from=builder /src/dist /opt/micro_sp_gui/dist

# The server takes its whole configuration from the environment (see the GUI's
# README), so compose can set everything without a command line here. The default
# --dist is relative, so it is spelled out rather than relying on the workdir.
ENV MICRO_SP_GUI_DIST=/opt/micro_sp_gui/dist

EXPOSE 8080

WORKDIR /opt/micro_sp_gui
CMD ["micro_sp_gui"]
