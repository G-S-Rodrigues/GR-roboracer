#!/usr/bin/env bash
# GPU-3010: the gpu image compiles and runs a CUDA kernel, the underlay
# resolves, and the gym's JAX stays on the CPU. Run it in the gpu service:
#   docker compose -f docker/docker-compose.yaml run --rm gpu \
#       bash /ws/docker/gpu/smoke.sh
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT

nvcc -std=c++20 -arch="sm_${CUDA_ARCH:-89}" "${here}/smoke.cu" \
    -o "${work}/smoke"
"${work}/smoke"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/racing_underlay/setup.bash
set -u
ros2 pkg prefix slam_toolbox

backend="$(python3 -c 'import jax; print(jax.default_backend())')"
if [ "${backend}" != "cpu" ]; then
    echo "gym JAX backend is ${backend}, expected cpu (gotcha #4)" >&2
    exit 1
fi
echo "jax backend: ${backend}"
echo "GPU-3010 PASS"
