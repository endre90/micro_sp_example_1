# The dice cell's two binaries:
#
#   runner   builds the model, seeds the state and runs micro_sp
#   cell     the simulated cell: dice, gripper, camera, disturbances
#
# Build context is the repository root, since the crate lives there:
#   docker build -f docker/dice_cell/docker/Dockerfile .

FROM rust:1.97-bookworm AS builder

RUN apt-get update && \
    apt-get install -y --no-install-recommends git pkg-config libssl-dev ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# micro_sp is declared with an ssh://git@github.com URL but is public, so fetch
# it over https; cargo's libgit2 fetcher ignores insteadOf, hence the git CLI.
RUN git config --global --add url."https://github.com/".insteadOf "ssh://git@github.com/" && \
    git config --global --add url."https://github.com/".insteadOf "git@github.com:"
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true

WORKDIR /build

# Dependencies first, against a stub source tree, so editing the crate does not
# rebuild them.
COPY dice_cell/Cargo.toml dice_cell/Cargo.lock ./
RUN mkdir -p src/bin && \
    : > src/lib.rs && \
    echo 'fn main() {}' > src/bin/runner.rs && \
    echo 'fn main() {}' > src/bin/cell.rs && \
    cargo build --release --bin runner --bin cell && \
    rm -rf src

COPY dice_cell/src ./src
RUN find src -name '*.rs' -exec touch {} + && \
    cargo build --release --bin runner --bin cell

FROM debian:bookworm-slim

RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates libssl3 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /build/target/release/runner /usr/local/bin/runner
COPY --from=builder /build/target/release/cell   /usr/local/bin/cell

WORKDIR /work
CMD ["runner"]
