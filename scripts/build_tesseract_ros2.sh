#!/usr/bin/env bash
# Build the tesseract_ros2 overlay (tesseract_msgs, tesseract_rosutils,
# tesseract_monitoring) from source against the conda-provided tesseract C++
# and a sourced ROS 2 distro.
#
# The base tesseract C++ now ships as the tesseract-robotics conda packages, so
# — unlike the old scripts/build_tesseract_cpp.sh — this does NOT build tesseract
# from source. It only builds the three tesseract_ros2 packages that conda does
# not ship, linking them against the conda tesseract so the bindings, the
# overlay, and the base C++ all share one tesseract.
#
# Usage (the ros-py312 env matches ROS Jazzy's python; the ros feature provides colcon):
#   source /opt/ros/<distro>/setup.bash
#   pixi run -e ros-py312 build-ros          # or: bash scripts/build_tesseract_ros2.sh
#   pixi run -e ros-py312 install            # builds the ROS 2 bindings against this overlay
#
# Output: ws/install (colcon merge-install). The `install` task adds it to
# CMAKE_PREFIX_PATH so find_package(tesseract_monitoring/rosutils/msgs) resolves.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$PROJECT_ROOT/ws"

# --- Preconditions ---------------------------------------------------------
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    echo "❌ No ROS 2 environment sourced."
    echo "   Run: source /opt/ros/<distro>/setup.bash   (e.g. jazzy)"
    exit 1
fi
if [[ -z "${CONDA_PREFIX:-}" ]]; then
    echo "❌ No conda/pixi env active."
    echo "   Run inside 'pixi shell' so the tesseract-robotics C++ libs are on CMAKE_PREFIX_PATH."
    exit 1
fi
echo "✓ ROS 2:  $AMENT_PREFIX_PATH"
echo "✓ conda:  $CONDA_PREFIX"

# --- Fetch tesseract_ros2 sources ------------------------------------------
# dependencies_ros.rosinstall lists a single repo, so clone it directly: the
# build then needs only git.
mkdir -p "$WORKSPACE_DIR/src"
ROS2_URI=$(awk '/uri:/{print $2; exit}' "$PROJECT_ROOT/dependencies_ros.rosinstall")
ROS2_VERSION=$(awk '/version:/{print $2; exit}' "$PROJECT_ROOT/dependencies_ros.rosinstall")
ROS2_NAME=$(awk '/local-name:/{print $2; exit}' "$PROJECT_ROOT/dependencies_ros.rosinstall")
ROS2_DEST="$WORKSPACE_DIR/src/$ROS2_NAME"
if [[ -d "$ROS2_DEST/.git" ]]; then
    echo "✓ $ROS2_NAME already present ($ROS2_DEST)"
else
    echo "Cloning $ROS2_NAME @ $ROS2_VERSION from $ROS2_URI"
    # Shallow-fetch the exact ref: works for a SHA, tag, or branch (GitHub allows
    # fetching an arbitrary commit). git clone --branch cannot take a raw SHA.
    git init -q "$ROS2_DEST"
    git -C "$ROS2_DEST" remote add origin "$ROS2_URI"
    git -C "$ROS2_DEST" fetch -q --depth 1 origin "$ROS2_VERSION"
    git -C "$ROS2_DEST" checkout -q FETCH_HEAD
fi

# --- Build only the packages we bind ---------------------------------------
# --packages-up-to pulls tesseract_msgs (a dependency) automatically and leaves
# the rviz / qt / example packages in the repo unbuilt. The base tesseract C++
# is found via CONDA_PREFIX; ROS 2 (rclcpp, *_msgs) via the sourced distro.
#
# The visibility flags MUST match add_tesseract_nanobind_extension() in
# CMakeLists.txt (default visibility, inlines not hidden) so typeid() compares
# equal across the binding/overlay .so boundary.
cd "$WORKSPACE_DIR"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LIBRARY_PATH="$CONDA_PREFIX/lib:${LIBRARY_PATH:-}"
echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

# colcon comes from the ros-py312 env (ros feature), whose python matches ROS Jazzy's
# 3.12 — so rosidl message generation and ament run under a consistent interpreter
# and no Python3 pinning / system-colcon fallback is needed.
colcon build \
    --merge-install \
    --packages-up-to tesseract_monitoring tesseract_rosutils \
    --event-handlers console_cohesion+ \
    --cmake-force-configure \
    --cmake-args \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_CXX_VISIBILITY_PRESET=default \
        -DCMAKE_VISIBILITY_INLINES_HIDDEN=OFF \
        -DBUILD_TESTING=OFF \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        "-DCMAKE_INSTALL_RPATH=\$ORIGIN:$CONDA_PREFIX/lib"

echo ""
echo "✓ tesseract_ros2 overlay built -> $WORKSPACE_DIR/install"
echo "  Next: pixi run -e ros-py312 install"
