# The UR5, kinematically: https://github.com/sequenceplanner/simple_robot_simulator
#
# Also carries the UR description (URDF + meshes), taken from ur_redis_driver's
# repository, at /opt/shared/ur_description. The compose file mounts a named
# volume there; Docker seeds an empty named volume from the image, and RViz
# mounts the same volume - so the mesh paths the simulator puts in its frames'
# metadata resolve to the same files in both containers.

FROM rust:1.97-bookworm AS builder

RUN apt-get update && \
    apt-get install -y --no-install-recommends git pkg-config libssl-dev ca-certificates && \
    rm -rf /var/lib/apt/lists/*

RUN git config --global --add url."https://github.com/".insteadOf "ssh://git@github.com/" && \
    git config --global --add url."https://github.com/".insteadOf "git@github.com:"
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true

ARG SIM_GIT_URL=https://github.com/sequenceplanner/simple_robot_simulator.git
ARG SIM_GIT_REF=master
ARG UR_REDIS_DRIVER_GIT_URL=https://github.com/sequenceplanner/ur_redis_driver.git
ARG UR_REDIS_DRIVER_GIT_REF=v0.3.1

# Bump to force a re-clone: --build-arg CACHE_BUST=$(date +%s)
ARG CACHE_BUST=0

RUN git clone "$SIM_GIT_URL" /src && cd /src && git checkout "$SIM_GIT_REF"
RUN git clone --depth 1 --branch "$UR_REDIS_DRIVER_GIT_REF" "$UR_REDIS_DRIVER_GIT_URL" /ur_redis_driver

WORKDIR /src
RUN cargo build --release --bin simple_robot_simulator

FROM debian:bookworm-slim

RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates libssl3 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /src/target/release/simple_robot_simulator /usr/local/bin/simple_robot_simulator
COPY --from=builder /ur_redis_driver/src/ur_description /opt/shared/ur_description

ENV DESCRIPTION_DIR=/opt/shared/ur_description \
    ROBOT_MODEL=ur5 \
    ROBOT_ID=r1

CMD ["simple_robot_simulator"]
