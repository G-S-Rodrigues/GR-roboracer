# syntax=docker/dockerfile:1.7
#
# GR-roboracer multi-stage image. See docs/adr/0002-development-environment.md.
#
#   base      ros:jazzy-ros-base + apt dependencies
#   underlay  third-party ROS sources built into /opt/racing_underlay
#   dev       underlay + toolchain; where all ROS nodes and colcon builds run
#   gpu       dev + CUDA 12.8 (nvcc, cudart) for GPU ROS nodes; the gym stays on
#             CPU; owner's machine only, never built in CI
#   fast-sim  python + JAX/CUDA + f1tenth_gym_jax. NO ROS.
#   runtime   ros:jazzy-ros-core + underlay + installed workspace, no toolchain
#
# `uv` (JAX) and `colcon` (ROS) live in separate images on purpose: mixing a
# uv-managed venv with ament's Python discovery is a known fight.

# Pinned upstream revisions. f1tenth_gym_jax declares jax-pf with NO rev, so
# pinning the gym alone is not a pin -- jax_pf supplies the ray marching behind
# /scan. Both SHAs come from the Step 1 spike (.scratch/.../spike-gym-jax.md).
ARG GYM_JAX_SHA=1b4eb3f5161756bb925987753b965b549097742f
ARG JAX_PF_SHA=1b1417d7d2afbf24a9c6594195c2e872a6b4460a
# sha256 of NVIDIA's cuda-keyring_1.1-1_all.deb (ubuntu2404), used by `gpu`.
ARG CUDA_KEYRING_SHA256=d2a6b11c096396d868758b86dab1823b25e14d70333f1dfa74da5ddaf6a06dba


# --------------------------------------------------------------------------
FROM ros:jazzy-ros-base AS base
ARG TARGETARCH
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ros-jazzy-asio-cmake-module is listed explicitly because rosdep does NOT
# resolve it: it is a transitive export dependency of serial_driver, and
# vesc_driver fails to configure without it.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        python3-vcstool \
        ros-jazzy-asio-cmake-module \
        ros-jazzy-ackermann-msgs \
        ros-jazzy-joy-teleop \
        ros-jazzy-key-teleop \
        ros-jazzy-teleop-tools \
        ros-jazzy-urg-node \
        ros-jazzy-rosbridge-server \
        ros-jazzy-nav2-amcl \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*


# --------------------------------------------------------------------------
FROM base AS underlay

# Underlay in the image, overlay in the mount: third-party packages are built
# once at image-build time so `colcon build` after an edit takes seconds and
# never rebuilds slam_toolbox.
COPY vcs/third_party.repos /tmp/third_party.repos
RUN mkdir -p /opt/racing_underlay_src \
    && vcs import /opt/racing_underlay_src < /tmp/third_party.repos

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /opt/racing_underlay_src --ignore-src -y -r \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
        --base-paths /opt/racing_underlay_src \
        --install-base /opt/racing_underlay \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/racing_underlay_src/../build /opt/racing_underlay_src/../log


# --------------------------------------------------------------------------
FROM underlay AS dev

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        build-essential gdb \
        clang clang-format clang-tidy \
        cmake ninja-build \
        git curl wget less \
        pybind11-dev python3-pybind11 \
        python3-pip python3-pytest python3-pytest-cov \
        libgtest-dev libgmock-dev \
        ros-dev-tools \
        ros-jazzy-rviz2 \
        ros-jazzy-launch-testing ros-jazzy-launch-testing-ament-cmake \
        ros-jazzy-ament-cmake-clang-format \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 marks the system interpreter externally-managed (PEP 668).
# This is a disposable container image, so installing into it is intended.
# The tools are a pinned closure installed --no-deps (nothing resolves at
# build time; see docker/dev-tools-requirements.txt), and `pip3 check` fails
# the build if a pinned dist disagrees with Debian's. It must stay before the
# gym install below, whose jax-pf override would fail it.
COPY docker/dev-tools-requirements.txt /tmp/dev-tools-requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages --no-deps \
        -r /tmp/dev-tools-requirements.txt \
    && pip3 check \
    && rm /tmp/dev-tools-requirements.txt

# uv is present in dev for tools/ and sim/ work, but sim/ itself runs in
# fast-sim: nothing here creates a venv that ament could discover.
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh

ARG GYM_JAX_SHA
COPY docker/gym-cpu-overrides.txt /tmp/gym-cpu-overrides.txt
# The ROS adapter is the one deliberate JAX-in-dev exception. Install the
# pinned CPU backend into the system interpreter used by rclpy; fast-sim keeps
# the CUDA environment isolated in its own image.
RUN uv pip install --system \
        --break-system-packages \
        --override /tmp/gym-cpu-overrides.txt \
        "f1tenth-gym-jax @ git+https://github.com/f1tenth/f1tenth_gym_jax.git@${GYM_JAX_SHA}" \
    && rm /tmp/gym-cpu-overrides.txt

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
WORKDIR /ws


# --------------------------------------------------------------------------
# `gpu` derives from `dev` by owner ruling (2026-10-25, ADR 0002's amendment):
# the owner develops, tests and runs the simulators here. `dev` must never
# derive from `gpu` (gotcha #4). The gym's JAX stays on CPU: it is inherited
# from `dev`, and nothing here installs a Python package (CHECK-1910).
FROM dev AS gpu
ARG CUDA_KEYRING_SHA256
COPY docker/gpu-apt-pins.txt /tmp/gpu-apt-pins.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    curl -fsSLo /tmp/cuda-keyring.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && echo "${CUDA_KEYRING_SHA256}  /tmp/cuda-keyring.deb" | sha256sum -c - \
    && dpkg -i /tmp/cuda-keyring.deb \
    && apt-get update \
    && grep -v '^#' /tmp/gpu-apt-pins.txt | grep . | sort > /tmp/gpu-pins.list \
    && xargs -a /tmp/gpu-pins.list apt-get install -y --no-install-recommends \
    && dpkg-query -W -f='${db:Status-Abbrev} ${Package}=${Version}\n' 'cuda-*' | grep '^ii ' | cut -c4- | sort > /tmp/gpu-installed.list \
    && diff /tmp/gpu-pins.list /tmp/gpu-installed.list \
    && rm -f /tmp/cuda-keyring.deb /tmp/gpu-pins.list /tmp/gpu-installed.list /tmp/gpu-apt-pins.txt
ENV PATH=/usr/local/cuda-12.8/bin:${PATH} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility


# --------------------------------------------------------------------------
FROM python:3.12-slim AS fast-sim
ARG GYM_JAX_SHA
ARG JAX_PF_SHA

# No ROS in this stage, by design.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates libyaml-cpp0.8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh

COPY sim/pyproject.toml sim/uv.lock /opt/gr-roboracer-sim/

# Two overrides, both required:
#
#  1. jax-pf is declared by f1tenth_gym_jax with NO rev. Pinning it forces the
#     resolved revision, so two builds of the "same" pinned gym SHA cannot
#     produce different LiDAR ray marching.
#
#  2. jax>=0.7.2 reproduces upstream's own `[tool.uv] override-dependencies`.
#     jax_pf still declares jax[cuda12]>=0.6.1,<0.7, which is unsatisfiable
#     against the gym's jax>=0.7.2,<0.8. Upstream resolves this with a
#     pyproject override -- but [tool.uv] applies only when uv operates ON that
#     project, NOT when it is installed as a dependency. Without this line the
#     resolve fails outright; passing only override (1) *replaces* upstream's
#     override rather than adding to it.
RUN uv sync --project /opt/gr-roboracer-sim --frozen --no-install-project
ENV PATH=/opt/gr-roboracer-sim/.venv/bin:${PATH}

# Pre-seed the map directory. Track.from_track_name downloads missing maps over
# PLAIN HTTP from api.f1tenth.org at env-construction time; an unseeded image
# reaches the network on first use and CI inherits a dependency nobody chose.
# Only Spielberg ships in-tree, so this seeds what upstream bundles.
ENV F1TENTH_GYM_JAX_MAP_DIR=/opt/f1tenth_gym_jax_maps
RUN git clone --filter=blob:none --no-checkout \
        https://github.com/f1tenth/f1tenth_gym_jax.git /tmp/gym \
    && git -C /tmp/gym checkout "${GYM_JAX_SHA}" -- maps \
    && mkdir -p "${F1TENTH_GYM_JAX_MAP_DIR}" \
    && cp -r /tmp/gym/maps/. "${F1TENTH_GYM_JAX_MAP_DIR}/" \
    && rm -rf /tmp/gym

WORKDIR /ws
CMD ["python"]


# --------------------------------------------------------------------------
FROM ros:jazzy-ros-core AS runtime
ENV DEBIAN_FRONTEND=noninteractive
COPY --from=underlay /opt/racing_underlay /opt/racing_underlay
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
WORKDIR /ws
