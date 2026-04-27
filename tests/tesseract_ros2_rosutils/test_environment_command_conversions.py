"""Roundtrip tests for tesseract_environment::Command <-> EnvironmentCommand.

Covers the high-leverage subset of command types whose msg fields are
bound. Limit-change commands and kinematics-info / contact-manager-plugin
commands are deliberately not covered (their msg fields rely on types we
chose not to bind in this PR — see the layer-6 comment in the bindings).
"""

import pytest

pytest.importorskip("tesseract_robotics.tesseract_ros2_rosutils")

from tesseract_robotics import tesseract_ros2_rosutils as ru  # noqa: E402
from tesseract_robotics.tesseract_common import GeneralResourceLocator  # noqa: E402
from tesseract_robotics.tesseract_environment import (  # noqa: E402
    AddLinkCommand,
    AddSceneGraphCommand,
    ChangeJointOriginCommand,
    Environment,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType_ADD,
    MoveJointCommand,
    RemoveJointCommand,
    RemoveLinkCommand,
)
from tesseract_robotics.tesseract_common import (  # noqa: E402
    AllowedCollisionMatrix,
    Isometry3d,
)
from tesseract_robotics.tesseract_scene_graph import (  # noqa: E402
    Joint,
    JointType,
    Link,
)

SIMPLE_URDF = """
<robot name="test_robot" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">
  <link name="world"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="fixed">
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <joint name="joint2" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
"""


@pytest.fixture
def env():
    e = Environment()
    assert e.init(SIMPLE_URDF, GeneralResourceLocator())
    return e


def test_add_link_command_to_msg():
    link = Link("test_link")
    cmd = AddLinkCommand(link)
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.ADD_LINK
    assert msg.add_link.name == "test_link"
    assert msg.add_replace_allowed is False


def test_add_link_command_roundtrip():
    link = Link("roundtrip_link")
    cmd = AddLinkCommand(link, False)
    msg = ru.command_to_msg(cmd)
    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, AddLinkCommand)
    assert cmd2.getLink().getName() == "roundtrip_link"


def test_add_link_with_joint_roundtrip():
    link = Link("child_link")
    joint = Joint("new_joint")
    joint.type = JointType.FIXED
    joint.parent_link_name = "world"
    joint.child_link_name = "child_link"
    cmd = AddLinkCommand(link, joint)
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.ADD_LINK
    assert msg.add_link.name == "child_link"
    assert msg.add_joint.name == "new_joint"
    assert msg.add_joint.parent_link_name == "world"
    assert msg.add_joint.child_link_name == "child_link"

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, AddLinkCommand)
    assert cmd2.getLink().getName() == "child_link"
    assert cmd2.getJoint() is not None
    assert cmd2.getJoint().getName() == "new_joint"


def test_remove_link_command_roundtrip():
    cmd = RemoveLinkCommand("link2")
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.REMOVE_LINK
    assert msg.remove_link == "link2"

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, RemoveLinkCommand)
    assert cmd2.getLinkName() == "link2"


def test_remove_joint_command_roundtrip():
    cmd = RemoveJointCommand("joint2")
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.REMOVE_JOINT
    assert msg.remove_joint == "joint2"

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, RemoveJointCommand)
    assert cmd2.getJointName() == "joint2"


def test_move_joint_command_roundtrip():
    cmd = MoveJointCommand("joint2", "world")
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.MOVE_JOINT
    assert msg.move_joint_name == "joint2"
    assert msg.move_joint_parent_link == "world"

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, MoveJointCommand)


def test_change_joint_origin_command_roundtrip():
    iso = Isometry3d.Identity()
    cmd = ChangeJointOriginCommand("joint1", iso)
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.CHANGE_JOINT_ORIGIN
    assert msg.change_joint_origin_name == "joint1"
    # default-init Pose: position (0,0,0), orientation (0,0,0,1) — matches Identity
    assert msg.change_joint_origin_pose.orientation.w == pytest.approx(1.0)

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, ChangeJointOriginCommand)


def test_modify_allowed_collisions_roundtrip():
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("link1", "link2", "test_reason")
    cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType_ADD)
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.MODIFY_ALLOWED_COLLISIONS
    assert len(msg.modify_allowed_collisions) >= 1
    entry = msg.modify_allowed_collisions[0]
    assert {entry.link_1, entry.link_2} == {"link1", "link2"}
    assert entry.reason == "test_reason"

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, ModifyAllowedCollisionsCommand)


def test_add_scene_graph_command_roundtrip(env):
    sub_sg = env.getSceneGraph().clone()
    cmd = AddSceneGraphCommand(sub_sg, "prefix_")
    msg = ru.command_to_msg(cmd)
    assert msg.command == ru.EnvironmentCommand.ADD_SCENE_GRAPH
    assert msg.scene_graph_prefix == "prefix_"
    # Scene graph should have all our links + joints
    link_names = {link.name for link in msg.scene_graph.links}
    assert {"world", "link1", "link2"}.issubset(link_names)
    joint_names = {joint.name for joint in msg.scene_graph.joints}
    assert {"joint1", "joint2"}.issubset(joint_names)

    cmd2 = ru.msg_to_command(msg)
    assert isinstance(cmd2, AddSceneGraphCommand)


def test_commands_vector_roundtrip():
    cmds = [
        AddLinkCommand(Link("vec_link_1"), False),
        RemoveJointCommand("joint2"),
        AddLinkCommand(Link("vec_link_2"), False),
    ]
    msgs = ru.commands_to_msg(cmds, 0)
    assert len(msgs) == 3
    assert msgs[0].command == ru.EnvironmentCommand.ADD_LINK
    assert msgs[1].command == ru.EnvironmentCommand.REMOVE_JOINT
    assert msgs[2].command == ru.EnvironmentCommand.ADD_LINK

    cmds2 = ru.msgs_to_commands(msgs)
    assert len(cmds2) == 3
    assert isinstance(cmds2[0], AddLinkCommand)
    assert isinstance(cmds2[1], RemoveJointCommand)
    assert isinstance(cmds2[2], AddLinkCommand)
    assert cmds2[1].getJointName() == "joint2"


def test_empty_commands_vector():
    msgs = ru.commands_to_msg([], 0)
    assert len(msgs) == 0
    cmds = ru.msgs_to_commands([])
    assert len(cmds) == 0
