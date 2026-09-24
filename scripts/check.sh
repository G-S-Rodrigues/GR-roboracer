#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat >&2 <<'EOF'
Usage: check.sh --fast|--ci|--full|--nightly

  --fast    build, lint, tier 1. The pre-commit gate; keep it under ~45s.
  --ci      --fast plus tier 2 and one seeded tier-3 run. The PR gate.
  --full    tiers 0-4. The definition of done, and deliberately not every
            test that exists (ADR 0005).
  --nightly --full plus tier 5: the cross-product of tracks,
            implementations and seeds. Minutes to hours; it runs on a
            schedule, never in the development loop.
EOF
}

if [[ $# -ne 1 ]] ||
    [[ "$1" != "--fast" && "$1" != "--ci" && "$1" != "--full" &&
    "$1" != "--nightly" ]]; then
    usage
    exit 2
fi

# The one tier-3 case the PR gate runs: a full seeded lap through the real
# launch. It is the cheapest test that would notice the whole vertical slice
# coming apart, which is what a three-minute gate is for.
ci_system_test="tests/system/test_sim_3020_seeded_lap_completes.py"

mode="$1"
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# A process started with `docker exec` does not inherit environment changes
# made by the container entrypoint before it execs the long-running command.
# Make this documented entry point self-contained for both attached shells and
# direct execution in the persistent development container.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/racing_underlay/setup.bash ]]; then
    source /opt/racing_underlay/setup.bash
fi
set -u

echo "==> build"
colcon build \
    --base-paths ros_ws/src \
    --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the overlay only after the build that produced it. Sourcing it before
# made every plain-python3 step below (sim/tests, the system tests) depend on an
# install/ left over from some earlier build: on a fresh checkout there is none,
# and racing_common's Python module was never importable.
set +u
source "$repo_root/install/setup.bash"
set -u

echo "==> clang-format"
mapfile -d '' cpp_files < <(
    find ros_ws/src -type f \
        \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' \
        -o -name '*.h' -o -name '*.hh' -o -name '*.hpp' -o -name '*.hxx' \) \
        -print0
)
if (( ${#cpp_files[@]} )); then
    clang-format --dry-run --Werror "${cpp_files[@]}"
fi

echo "==> clang-tidy"
while IFS= read -r -d '' compilation_database; do
    run-clang-tidy \
        -p "$(dirname "$compilation_database")" \
        -config-file "$repo_root/.clang-tidy"
done < <(find build -name compile_commands.json -type f -print0)

echo "==> ruff"
ruff check sim tools tests scripts ros_ws/src
ruff format --check sim tools tests scripts ros_ws/src

echo "==> ROS package tests"
if [[ "$mode" == "--fast" ]]; then
    colcon test \
        --event-handlers console_cohesion+ \
        --return-code-on-test-failure \
        --ctest-args -L 'tier1|gtest|lint' --output-on-failure
else
    # --ci and --full both run every package test: tier 2 costs ~5s, so
    # splitting it out would buy nothing and leave a hole in the PR gate.
    colcon test \
        --event-handlers console_cohesion+ \
        --return-code-on-test-failure \
        --ctest-args --output-on-failure
fi
colcon test-result --verbose

if find sim/tests -type f -name 'test_*.py' -print -quit 2>/dev/null | grep -q .; then
    echo "==> simulator unit tests"
    python3 -m pytest sim/tests
fi

if [[ "$mode" == "--ci" ]] && [[ -f "$ci_system_test" ]]; then
    echo "==> system smoke test"
    python3 -m pytest "$ci_system_test"
fi

if [[ "$mode" == "--full" || "$mode" == "--nightly" ]]; then
    if find tests/system -type f -name 'test_*.py' -print -quit | grep -q .; then
        echo "==> system tests"
        python3 -m pytest tests/system
    fi

    if find tests/acceptance -type f -name '*.robot' -print -quit | grep -q .; then
        echo "==> acceptance tests"
        robot --pythonpath tests/lib --outputdir log/robot tests/acceptance
    fi
fi

# Tier 5: the cross-product ADR 0005 keeps out of the gate, so that --fast,
# --ci and --full do not grow as implementations accumulate. Nothing here is
# part of "done"; what it buys is that a regression only the combination of
# track, implementation and seed shows up in is visible the next morning.
if [[ "$mode" == "--nightly" ]]; then
    if find tests/nightly -type f -name 'test_*.py' -print -quit | grep -q .; then
        echo "==> nightly cross-product"
        python3 -m pytest tests/nightly
    fi
fi

echo "==> ${mode#--} checks passed"
