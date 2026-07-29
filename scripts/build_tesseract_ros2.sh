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
# Usually you do not run this directly: scripts/install.sh (the `install` task) calls it
# when a ROS 2 distro is sourced and ws/install is absent. Run it explicitly to rebuild
# the overlay on its own, e.g. after bumping dependencies_ros.rosinstall:
#
#   source /opt/ros/<distro>/setup.bash
#   pixi run build-ros
#
# The overlay is ROS-side software, so it is built with the ROS side's toolchain: the
# distro's own python and its colcon, not the pixi env's. That interpreter matches the
# distro by construction and already has the rosidl/ament python deps (empy, lark,
# catkin_pkg) from apt, so the pixi env needs no ROS python packages and no python pin.
# The CMake macro package the overlay build_depends on is built here too, from
# dependencies_ros.rosinstall. Only the conda tesseract C++ comes from the pixi env, via
# CMAKE_PREFIX_PATH. Override the interpreter with TESSERACT_ROS_PYTHON if your ROS 2 is
# not a system apt install.
#
# Output: ws/install (colcon merge-install). CMakeLists.txt adds that directory to
# CMAKE_PREFIX_PATH when it exists, so find_package(tesseract_monitoring/rosutils/msgs)
# resolves and the tesseract_ros2_* bindings switch on with no flag.

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

# --- Pick the ROS distro's python ------------------------------------------
# The distro puts its own packages on PYTHONPATH as <prefix>/lib/pythonX.Y/site-packages;
# take X.Y from there so the interpreter matches the distro exactly, and fall back to the
# default system python if that yields nothing.
if [[ -z "${TESSERACT_ROS_PYTHON:-}" ]]; then
    ROS_PY_VER=$(printf '%s' "${PYTHONPATH:-}" | tr ':' '\n' \
        | sed -n 's|.*/lib/python\([0-9][0-9.]*\)/site-packages/*$|\1|p' | head -1)
    if [[ -n "$ROS_PY_VER" && -x "/usr/bin/python$ROS_PY_VER" ]]; then
        TESSERACT_ROS_PYTHON="/usr/bin/python$ROS_PY_VER"
    else
        TESSERACT_ROS_PYTHON="/usr/bin/python3"
    fi
fi
ROS_PYTHON="$TESSERACT_ROS_PYTHON"

if [[ ! -x "$ROS_PYTHON" ]]; then
    echo "❌ ROS python not found at $ROS_PYTHON"
    echo "   Set TESSERACT_ROS_PYTHON to the interpreter your ROS 2 distro was built for."
    exit 1
fi
# colcon drives the build; empy/lark are imported by the rosidl generators. All three are
# apt packages alongside a system ROS 2 install, and all must be visible to THIS python.
for mod_pkg in colcon_core:python3-colcon-common-extensions em:python3-empy lark:python3-lark; do
    if ! "$ROS_PYTHON" -c "import ${mod_pkg%%:*}" >/dev/null 2>&1; then
        echo "❌ $ROS_PYTHON cannot import '${mod_pkg%%:*}', needed to build the overlay."
        echo "   Install it:  sudo apt install ${mod_pkg##*:}"
        exit 1
    fi
done
echo "✓ python:  $ROS_PYTHON ($("$ROS_PYTHON" --version))"

# --- Fetch the workspace sources -------------------------------------------
# Clone each repo in dependencies_ros.rosinstall directly, so the build needs only git
# (no vcstool). The awk emits one "local-name uri version" line per `- git:` block,
# flushing on `version:` because that is the last key in each block.
mkdir -p "$WORKSPACE_DIR/src"
while read -r name uri version; do
    dest="$WORKSPACE_DIR/src/$name"
    if [[ -d "$dest/.git" ]]; then
        echo "✓ $name already present ($dest)"
        continue
    fi
    echo "Cloning $name @ $version from $uri"
    # Shallow-fetch the exact ref: works for a SHA, tag, or branch (GitHub allows
    # fetching an arbitrary commit). git clone --branch cannot take a raw SHA.
    git init -q "$dest"
    git -C "$dest" remote add origin "$uri"
    git -C "$dest" fetch -q --depth 1 origin "$version"
    git -C "$dest" checkout -q FETCH_HEAD
done < <(awk '/local-name:/{n=$2} /uri:/{u=$2} /version:/{if(n&&u){print n, u, $2; n=""; u=""}}' \
    "$PROJECT_ROOT/dependencies_ros.rosinstall")

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

# rosidl/ament run their generators under CMake's discovered interpreter, and that
# discovery has to land on $ROS_PYTHON. Two things derail it, so pin the interpreter
# instead of trusting the search:
#   * CMake's FindPython3 defaults to Python3_FIND_VIRTUALENV=FIRST, so a stray
#     VIRTUAL_ENV in the caller's shell (e.g. a leftover .venv) outranks everything else
#     — and that interpreter has no empy, so rosidl_adapter dies with
#     "No module named 'em'";
#   * a bare `python3` on PATH resolves to the pixi env when run under `pixi run`.
# ament reads PYTHON_EXECUTABLE, modern CMake modules read Python3_EXECUTABLE; set both.
# Invoking colcon as `python -m` rather than by name pins its interpreter the same way.
unset VIRTUAL_ENV

"$ROS_PYTHON" -m colcon build \
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
        -DPython3_EXECUTABLE="$ROS_PYTHON" \
        -DPYTHON_EXECUTABLE="$ROS_PYTHON" \
        "-DCMAKE_INSTALL_RPATH=\$ORIGIN:$CONDA_PREFIX/lib"

echo ""
echo "✓ tesseract_ros2 overlay built -> $WORKSPACE_DIR/install"
echo "  The tesseract_ros2_* bindings now build automatically: pixi run install"
