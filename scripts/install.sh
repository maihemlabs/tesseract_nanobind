#!/usr/bin/env bash
# Editable install of the bindings. This is `pixi run install`.
#
# ROS 2 needs no separate command or CMake flag. If a ROS 2 distro is sourced, the
# tesseract_ros2 overlay (tesseract_msgs, tesseract_rosutils, tesseract_monitoring —
# conda ships none of them) is source-built into ws/install first, and CMakeLists.txt
# auto-detects it from there and builds the two tesseract_ros2_* modules. With no ROS 2
# sourced this is a plain, ROS-free editable install. Same command either way:
#
#   pixi run install                                        # ROS-free
#   source /opt/ros/jazzy/setup.bash && pixi run install     # + ROS 2 bindings
#
# The overlay build uses the ROS distro's own toolchain (see build_tesseract_ros2.sh), so
# it works from any pixi env regardless of that env's python. See the ROS 2 section of
# README.md for the details.
#
# Escape hatch: export TESSERACT_NANOBIND_BUILD_ROS=OFF to force a ROS-free build in a
# shell that happens to have ROS sourced (=ON to hard-fail instead of silently skipping).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

ROS_MODE="${TESSERACT_NANOBIND_BUILD_ROS:-AUTO}"

if [[ "$ROS_MODE" != "OFF" && -n "${AMENT_PREFIX_PATH:-}" ]]; then
    if [[ -d "$PROJECT_ROOT/ws/install" ]]; then
        echo "✓ tesseract_ros2 overlay present (ws/install) — ROS 2 bindings will build"
    else
        echo "→ ROS 2 detected, tesseract_ros2 overlay missing — building it (one-time)"
        # build_tesseract_ros2.sh checks its own toolchain preconditions and reports the
        # apt package to install if anything is missing. To skip the ROS 2 bindings in a
        # ROS-sourced shell instead, use TESSERACT_NANOBIND_BUILD_ROS=OFF.
        bash "$SCRIPT_DIR/build_tesseract_ros2.sh"
    fi
fi

# The mode is passed through explicitly rather than left to the CMake default so that
# ON (hard-fail) and OFF (never) reach the build, and so the configure log records which
# mode the install ran under.
exec pip install -e "$PROJECT_ROOT" --no-build-isolation \
    --config-settings=cmake.define.TESSERACT_NANOBIND_BUILD_ROS="$ROS_MODE"
