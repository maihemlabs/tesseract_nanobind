"""Tests for tesseract_motion_planners_simple.

Covers the 2 profile bases and 7 concrete move profiles, the ProfileDictionary
registration helpers, and the generateInterpolatedProgram utility.
"""

import math

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLRealVectorPlanProfile,
    ProfileDictionary_addOMPLProfile,
)
from tesseract_robotics.tesseract_motion_planners_simple import (
    SimplePlannerCompositeProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerMoveProfile,
    generateInterpolatedProgram,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"


@pytest.fixture
def abb_irb2400_environment():
    """Load ABB IRB2400 robot environment for testing."""
    locator = GeneralResourceLocator()
    urdf_path = FilesystemPath(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf_path = FilesystemPath(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    )
    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator), "Failed to initialize ABB IRB2400"
    return t_env


# ---- Fixed-size profiles ---------------------------------------------------

FIXED_SIZE_CLASSES = [
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
]


@pytest.mark.parametrize("cls", FIXED_SIZE_CLASSES)
class TestFixedSizeProfiles:
    def test_default_construction(self, cls):
        p = cls()
        assert p is not None
        # C++ defaults: freespace_steps=10, linear_steps=10
        assert p.freespace_steps == 10
        assert p.linear_steps == 10

    def test_custom_construction(self, cls):
        p = cls(freespace_steps=25, linear_steps=7)
        assert p.freespace_steps == 25
        assert p.linear_steps == 7

    def test_member_assignment(self, cls):
        p = cls()
        p.freespace_steps = 42
        p.linear_steps = 3
        assert p.freespace_steps == 42
        assert p.linear_steps == 3

    def test_is_simple_move_profile(self, cls):
        p = cls()
        assert isinstance(p, SimplePlannerMoveProfile)


# ---- LVS profiles ----------------------------------------------------------

LVS_CLASSES = [
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
]


@pytest.mark.parametrize("cls", LVS_CLASSES)
class TestLVSProfiles:
    def test_default_construction(self, cls):
        p = cls()
        assert p is not None
        # C++ defaults: 5°, 0.1m, 5°, min_steps=1, max_steps=INT_MAX
        expected_rad = 5.0 * math.pi / 180.0
        assert p.state_longest_valid_segment_length == pytest.approx(expected_rad)
        assert p.translation_longest_valid_segment_length == pytest.approx(0.1)
        assert p.rotation_longest_valid_segment_length == pytest.approx(expected_rad)
        assert p.min_steps == 1
        # max_steps defaults to INT_MAX — just make sure it's big
        assert p.max_steps > 1000

    def test_custom_construction(self, cls):
        p = cls(
            state_longest_valid_segment_length=0.05,
            translation_longest_valid_segment_length=0.2,
            rotation_longest_valid_segment_length=0.1,
            min_steps=3,
            max_steps=99,
        )
        assert p.state_longest_valid_segment_length == pytest.approx(0.05)
        assert p.translation_longest_valid_segment_length == pytest.approx(0.2)
        assert p.rotation_longest_valid_segment_length == pytest.approx(0.1)
        assert p.min_steps == 3
        assert p.max_steps == 99

    def test_member_assignment(self, cls):
        p = cls()
        p.min_steps = 5
        p.max_steps = 50
        p.translation_longest_valid_segment_length = 0.25
        assert p.min_steps == 5
        assert p.max_steps == 50
        assert p.translation_longest_valid_segment_length == pytest.approx(0.25)

    def test_is_simple_move_profile(self, cls):
        p = cls()
        assert isinstance(p, SimplePlannerMoveProfile)


# ---- Composite base --------------------------------------------------------


class TestSimplePlannerCompositeProfile:
    def test_default_construction(self):
        p = SimplePlannerCompositeProfile()
        assert p is not None


# ---- ProfileDictionary helpers ---------------------------------------------


class TestProfileDictionaryHelpers:
    def test_add_move_profile(self):
        d = ProfileDictionary()
        profile = SimplePlannerFixedSizeMoveProfile(5, 8)
        d.addProfile("ns", "DEFAULT", profile)
        assert d.hasProfile(profile.getKey(), "ns", "DEFAULT") is True

    def test_add_composite_profile(self):
        d = ProfileDictionary()
        profile = SimplePlannerCompositeProfile()
        d.addProfile("ns", "DEFAULT", profile)
        assert d.hasProfile(profile.getKey(), "ns", "DEFAULT") is True


# ---- generateInterpolatedProgram -------------------------------------------


class TestSimplePlanner:
    """Test simple motion planner utilities."""

    def test_generate_interpolated_program(self, abb_irb2400_environment):
        """generateInterpolatedProgram over an OMPL-planned result."""
        t_env = abb_irb2400_environment

        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        t_env.setState([f"joint_{i + 1}" for i in range(6)], np.ones(6) * 0.1)

        # Single-waypoint freespace move.
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.0, 1.455)
            * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
        )
        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        # First solve with OMPL to get a valid result to interpolate.
        plan_profile = OMPLRealVectorPlanProfile()
        profiles = ProfileDictionary()
        ProfileDictionary_addOMPLProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

        request = PlannerRequest()
        request.instructions = program
        request.env = t_env
        request.profiles = profiles

        response = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE).solve(request)

        if response.successful:
            interpolated = generateInterpolatedProgram(response.results, t_env, 3.14, 1.0, 3.14, 10)
            assert interpolated is not None
