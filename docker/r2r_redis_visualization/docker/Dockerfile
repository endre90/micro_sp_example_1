# The Redis -> ROS visualization bridge:
# https://github.com/sequenceplanner/r2r_redis_visualization
#
# It reads the scenario transforms out of Redis and publishes /tf, /tf_static,
# /mesh_markers and /zone_markers for RViz to draw.

ARG ROS_DISTRO=lyrical

FROM ros:${ROS_DISTRO}-ros-base AS builder

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential ca-certificates clang curl git libclang-dev \
        libssl-dev pkg-config python3 && \
    rm -rf /var/lib/apt/lists/*

ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
        sh -s -- -y --default-toolchain 1.97.1 --profile minimal --no-modify-path

# `r2r` and `micro_sp` are declared with `ssh://git@github.com/...` URLs but are
# both public, so rewrite them to https rather than requiring a GitHub SSH key.
# Cargo's built-in libgit2 fetcher does not honour `insteadOf`, so it has to be
# told to shell out to the git CLI, which does.
# Both rewrites target the same config key, so the second one needs --add or it
# would overwrite the first instead of adding to it.
RUN git config --global --add url."https://github.com/".insteadOf "ssh://git@github.com/" && \
    git config --global --add url."https://github.com/".insteadOf "git@github.com:"
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true

ARG VIZ_GIT_URL=https://github.com/sequenceplanner/r2r_redis_visualization.git
ARG VIZ_GIT_REF=v0.4.0

# Bump to force a re-clone: --build-arg CACHE_BUST=$(date +%s)
ARG CACHE_BUST=0

RUN git clone "$VIZ_GIT_URL" /src && \
    cd /src && \
    git checkout "$VIZ_GIT_REF"

WORKDIR /src
# r2r generates its Rust message types from the *sourced* ROS environment, so
# the build has to happen with the underlay sourced.
RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cargo build --release

FROM ros:${ROS_DISTRO}-ros-base

COPY --from=builder /src/target/release/r2r_redis_visualization /usr/local/bin/r2r_redis_visualization

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["r2r_redis_visualization"]
