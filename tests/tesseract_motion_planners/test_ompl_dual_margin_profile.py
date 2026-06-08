"""Tests for OMPLRealVectorDualMarginMoveProfile.

This profile decouples the collision margin used for path *routing*
(routing_contact_manager_config) from the margin used for start/goal
*admission* (the inherited contact_manager_config). The override lives in
C++ (createCollisionStateValidator / createMotionValidator) and is dispatched
virtually by the unchanged base createSimpleSetup(), so the planning-loop
tests below are what actually prove the override is wired in.
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import ContactManagerConfig
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
    OMPLMoveProfile,
    OMPLRealVectorDualMarginMoveProfile,
    OMPLRealVectorMoveProfile,
    OMPLSolverConfig,
    ProfileDictionary_addOMPLMoveProfile,
    RRTConnectConfigurator,
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


def _make_program():
    """Build the freespace program used by the planning tests (wp1 -> wp2)."""
    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    wp1 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(0.8, -0.3, 1.455)
        * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
    )
    wp2 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(0.8, 0.3, 1.455)
        * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
    )

    start_instruction = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
        MoveInstructionType_FREESPACE,
        "DEFAULT",
    )
    plan_f1 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
        MoveInstructionType_FREESPACE,
        "DEFAULT",
    )

    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))
    return program


def _solve(t_env, profile):
    """Register `profile` and run the OMPL planner over the wp1->wp2 program."""
    # Stop at first solution to keep the test fast. A fresh OMPLSolverConfig has no
    # planners (the profile ctor normally installs them), so add one explicitly.
    solver = OMPLSolverConfig()
    solver.addPlanner(RRTConnectConfigurator())
    solver.planning_time = 2.0
    solver.optimize = False
    profile.solver_config = solver

    profiles = ProfileDictionary()
    ProfileDictionary_addOMPLMoveProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", profile)

    t_env.setState([f"joint_{i + 1}" for i in range(6)], np.ones(6) * 0.1)

    request = PlannerRequest()
    request.instructions = _make_program()
    request.env = t_env
    request.profiles = profiles

    return OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE).solve(request)


class TestDualMarginProfileConstruction:
    """Unit tests: construction, inheritance, and attribute wiring."""

    def test_construct(self):
        assert OMPLRealVectorDualMarginMoveProfile() is not None

    def test_is_subclass_of_base_profiles(self):
        profile = OMPLRealVectorDualMarginMoveProfile()
        assert isinstance(profile, OMPLRealVectorMoveProfile)
        assert isinstance(profile, OMPLMoveProfile)

    def test_inherits_base_attributes(self):
        profile = OMPLRealVectorDualMarginMoveProfile()
        assert hasattr(profile, "contact_manager_config")
        assert hasattr(profile, "collision_check_config")
        assert hasattr(profile, "solver_config")

    def test_has_routing_attribute(self):
        profile = OMPLRealVectorDualMarginMoveProfile()
        assert hasattr(profile, "routing_contact_manager_config")

    def test_routing_and_admission_are_independent(self):
        """Setting one margin must not affect the other."""
        profile = OMPLRealVectorDualMarginMoveProfile()
        profile.contact_manager_config = ContactManagerConfig(0.001)  # admission: tight
        profile.routing_contact_manager_config = ContactManagerConfig(0.05)  # routing: inflated

        assert profile.contact_manager_config.default_margin == pytest.approx(0.001)
        assert profile.routing_contact_manager_config.default_margin == pytest.approx(0.05)

    def test_exported_in_package_all(self):
        import tesseract_robotics.tesseract_motion_planners_ompl as ompl

        assert "OMPLRealVectorDualMarginMoveProfile" in ompl.__all__

    def test_registers_in_profile_dictionary(self):
        """The dual-margin profile upcasts to OMPLMoveProfile for ProfileDictionary."""
        profile = OMPLRealVectorDualMarginMoveProfile()
        profiles = ProfileDictionary()
        # Must not raise: exercises the shared_ptr upcast through OMPLMoveProfile.
        ProfileDictionary_addOMPLMoveProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", profile)


class TestDualMarginProfilePlanning:
    """Integration tests: the C++ override must run inside the planning loop."""

    def test_plan_succeeds_with_dual_margin_profile(self, abb_irb2400_environment):
        """A dual-margin profile with sane margins plans like the base profile.

        This exercises createSimpleSetup() -> (overridden) createCollisionStateValidator,
        proving the subclass compiles into the planning loop and routes successfully.
        """
        profile = OMPLRealVectorDualMarginMoveProfile()
        profile.routing_contact_manager_config = ContactManagerConfig(0.0)

        response = _solve(abb_irb2400_environment, profile)

        assert response.successful, f"OMPL planning failed: {response.message}"
        assert response.results is not None

    def test_routing_margin_governs_routing(self, abb_irb2400_environment):
        """An enormous routing margin must block routing while admission still passes.

        contact_manager_config (admission) stays tight, so the start/goal are admitted
        and createSimpleSetup() does not throw. routing_contact_manager_config is set so
        large that every state self-collides, so the routing validator rejects them and
        the planner cannot connect -> unsuccessful. This is only possible if the override
        actually feeds routing_contact_manager_config into the routing validator.
        """
        profile = OMPLRealVectorDualMarginMoveProfile()
        profile.contact_manager_config = ContactManagerConfig(0.0)  # admission: truthful
        profile.routing_contact_manager_config = ContactManagerConfig(5.0)  # routing: everything collides

        response = _solve(abb_irb2400_environment, profile)

        assert not response.successful, (
            "Plan unexpectedly succeeded with a 5 m routing margin; the routing validator "
            "is not honoring routing_contact_manager_config."
        )

    def test_tight_routing_still_succeeds_for_same_setup(self, abb_irb2400_environment):
        """Control for the test above: identical setup, only the routing margin shrinks."""
        profile = OMPLRealVectorDualMarginMoveProfile()
        profile.contact_manager_config = ContactManagerConfig(0.0)
        profile.routing_contact_manager_config = ContactManagerConfig(0.0)

        response = _solve(abb_irb2400_environment, profile)

        assert response.successful, f"OMPL planning failed: {response.message}"
