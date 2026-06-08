#!/usr/bin/env bash
# Editable-install the bindings with the ROS 2 modules enabled.
#
# Puts the tesseract_ros2 overlay built by build_tesseract_ros2.sh (ws/install)
# plus the conda env on CMAKE_PREFIX_PATH so the CMake build can find_package
# tesseract_monitoring / tesseract_rosutils / tesseract_msgs, then runs the
# editable pip install with -DTESSERACT_NANOBIND_BUILD_ROS=ON.
#
# Run from the ros-py312 env with a sourced ROS 2 distro:
#   source /opt/ros/<distro>/setup.bash && pixi run -e ros-py312 install
# (the ros-feature `install` overrides the default and depends on build-ros). The env
# python matches ROS Jazzy's 3.12,
# so ament_cmake/rosidl's find_package() helpers resolve from the sourced ROS — no
# extra pip installs needed.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    echo "❌ No ROS 2 environment sourced. Run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi
if [[ ! -d "$PROJECT_ROOT/ws/install" ]]; then
    echo "❌ ws/install not found — run 'pixi run -e ros-py312 build-ros' first."
    exit 1
fi

# Overlay first, then conda, then whatever ROS / system paths are already set.
export CMAKE_PREFIX_PATH="$PROJECT_ROOT/ws/install:${CONDA_PREFIX:-}:${CMAKE_PREFIX_PATH:-}"
echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

exec pip install -e "$PROJECT_ROOT" --no-build-isolation \
    --config-settings=cmake.define.TESSERACT_NANOBIND_BUILD_ROS=ON
