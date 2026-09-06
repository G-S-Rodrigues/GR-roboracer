#!/usr/bin/env bash

set -euo pipefail

usage() {
    echo "Usage: $0 --fast|--full" >&2
}

if [[ $# -ne 1 ]] || [[ "$1" != "--fast" && "$1" != "--full" ]]; then
    usage
    exit 2
fi

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
if [[ -f "$repo_root/install/setup.bash" ]]; then
    source "$repo_root/install/setup.bash"
fi
set -u

echo "==> build"
colcon build \
    --base-paths ros_ws/src \
    --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

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
ruff check sim tools tests ros_ws/src
ruff format --check sim tools tests ros_ws/src

echo "==> ROS package tests"
if [[ "$mode" == "--fast" ]]; then
    colcon test \
        --event-handlers console_cohesion+ \
        --return-code-on-test-failure \
        --ctest-args -L 'tier1|gtest|lint' --output-on-failure
else
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

if [[ "$mode" == "--full" ]]; then
    if find tests/system -type f -name 'test_*.py' -print -quit | grep -q .; then
        echo "==> system tests"
        python3 -m pytest tests/system
    fi

    if find tests/acceptance -type f -name '*.robot' -print -quit | grep -q .; then
        echo "==> acceptance tests"
        robot tests/acceptance
    fi
fi

echo "==> ${mode#--} checks passed"
