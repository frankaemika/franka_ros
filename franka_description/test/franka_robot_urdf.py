import os
import sys
import subprocess
from unittest import TestCase
from parameterized import parameterized_class
from .urdf_test_case import UrdfTestCase, PKG
from urdf_parser_py.urdf import URDF, Mesh, Cylinder, Sphere


@parameterized_class(('robot',), [("panda",), ("fr3",)])
class FrankaRobotUrdfTest(UrdfTestCase):

    @property
    def file(self):
        return os.path.join(self.robot, self.robot + '.urdf.xacro')

    def test_generate_urdf_without_xacro_args_is_possible(self):
        self.xacro(self.file)  # does not throw

    def test_generate_urdf_without_xacro_args_contains_link0_up_to_link8(self):
        arm_id = self.robot
        urdf = self.xacro(self.file)
        for i in range(0, 9):
            self.assertContainsLink(urdf, '%s_link%s' % (arm_id, i))

    def test_generate_urdf_without_xacro_args_contains_joint1_up_to_joint8(self):
        arm_id = self.robot
        urdf = self.xacro(self.file)
        for i in range(1, 9):
            joint = 'panda_joint%s' % i
            self.assertContainsJoint(urdf, '%s_joint%s' % (arm_id, i))

    def test_generate_urdf_without_xacro_args_dont_use_gripper(self):
        urdf = self.xacro(self.file)
        self.assertFalse(
            self.links_with(urdf, lambda link: 'hand' in link),  # should be empty
            'Found one or more links containing "hand", probably URDF contains a franka_gripper'
        )
        self.assertFalse(
            self.joints_with(urdf, lambda joint: 'hand' in joint),  # should be empty
            'Found one or more joints containing "hand", probably URDF contains a franka_gripper'
        )

    def test_generate_urdf_without_xacro_args_uses_fine_collision_models(self):
        arm_id = self.robot
        urdf = self.xacro(self.file)
        for i in range(0, 8):
            link = '%s_link%s' % (arm_id, i)
            collisions = self.collision_geometries(urdf, link)
            self.assertGreaterEqual(len(collisions), 1, "Link '%s' does not have any collision meshes assigned to it" % link)
            self.assertIsInstance(
                collisions[0], Mesh,
                "Link '%s' is expected to have mesh geometries in <collision> not '%s'" % (link, collisions[0].__class__.__name__)
            )

    def test_generate_urdf_without_xacro_args_uses_coarse_collision_models_for_sc_links(self):
        urdf = self.xacro(self.file)
        for name in urdf.link_map:
            if not name.endswith('_sc'): continue
            geometries = self.collision_geometries(urdf, name)
            for geometry in geometries:
                self.assertIsInstance(
                    geometry, (Cylinder, Sphere),
                    "Link '%s' is expected to define only a capsule <collision> geometry (made from Cylinders and Spheres, not '%s')" % (name, geometry.__class__.__name__)
                )

    def test_generate_urdf_without_xacro_args_doesnt_insert_inertial_tags_for_any_link(self):
        urdf = self.xacro(self.file)
        for name, link in urdf.link_map.items():
            self.assertIsNone(
                link.inertial,
                "Link '%s' is expected to have no <inertial> defined but actually has one:\n%s" % (name, link.inertial)
            )

    def test_generate_urdf_with_hand_but_not_gazebo_doesnt_insert_inertial_tags_for_any_link(self):
        urdf = self.xacro(self.file, args='hand:=true')
        for name, link in urdf.link_map.items():
            self.assertIsNone(
                link.inertial,
                "Link '%s' is expected to have no <inertial> defined but actually has one:\n%s" % (name, link.inertial)
            )

    def test_custom_arm_id_renames_links(self):
        arm_id = 'foo'
        urdf = self.xacro(self.file, args='arm_id:=%s' % arm_id)
        for link in urdf.link_map.keys():
            self.assertIn(arm_id, link)

    def test_custom_arm_id_renames_joints(self):
        arm_id = 'foo'
        urdf = self.xacro(self.file, args='arm_id:=%s' % arm_id)
        for joint in urdf.joint_map.keys():
            self.assertIn(arm_id, joint)

    def test_generate_urdf_with_hand_puts_franka_gripper_into_urdf(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='hand:=true')
        for name in ['hand', 'hand_tcp', 'leftfinger', 'rightfinger']:
            link = '%s_%s' % (arm_id, name)
            self.assertContainsLink(urdf, link)

        for name in ['hand_joint', 'hand_tcp_joint', 'finger_joint1', 'finger_joint2']:
            joint = '%s_%s' % (arm_id, name)
            self.assertContainsJoint(urdf, joint)

    def test_custom_arm_id_with_hand_renames_hand_joints_and_links(self):
        arm_id = 'foo'
        urdf = self.xacro(self.file, args='arm_id:=%s hand:=true' % arm_id)
        for name in ['hand', 'hand_tcp', 'leftfinger', 'rightfinger']:
            link = '%s_%s' % (arm_id, name)
            self.assertContainsLink(urdf, link)

        for name in ['hand_joint', 'hand_tcp_joint', 'finger_joint1', 'finger_joint2']:
            joint = '%s_%s' % (arm_id, name)
            self.assertContainsJoint(urdf, joint)

    def test_gazebo_arg_will_add_top_level_world_link(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='gazebo:=true')

        # Check if the world link exists
        self.assertEqual('world', urdf.get_root())

        # Check if robot is directly connected to the world link
        self.assertJointBetween(urdf, 'world', arm_id + '_link0', type="fixed")

    def test_gazebo_arg_will_insert_gazebo_ros_control_plugin(self):
        urdf = self.xacro(self.file, args='gazebo:=true')

        for gazebo in urdf.gazebos:
            for child in gazebo:
                if child.tag == 'plugin':
                    self.assertIn('name', child.attrib, '<plugin> tag does not have a "name" attribute')
                    self.assertEqual('gazebo_ros_control', child.attrib['name'])
                    # Successfully found plugin, break out of all loops
                    break
            else:
                # Inner loop not broken, continue with next gazebo tag
                continue
            # Inner loop was broken, break also outer loop
            break
        else:
            # Outer loop not broken -> no valid plugin found!
            self.fail('No <plugin name="gazebo_ros_control"> found in URDF')

    def test_gazebo_arg_will_insert_position_interfaces(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='gazebo:=true')
        for joint in ['%s_joint%s' % (arm_id, i) for i in range(1,8)]:
            self.assertJointHasTransmission(urdf, joint, 'hardware_interface/PositionJointInterface')

    def test_gazebo_arg_will_insert_velocity_interfaces(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='gazebo:=true')
        for joint in ['%s_joint%s' % (arm_id, i) for i in range(1,8)]:
            self.assertJointHasTransmission(urdf, joint, 'hardware_interface/VelocityJointInterface')

    def test_gazebo_arg_will_insert_effort_interfaces(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='gazebo:=true')
        for joint in ['%s_joint%s' % (arm_id, i) for i in range(1,8)]:
            self.assertJointHasTransmission(urdf, joint, 'hardware_interface/EffortJointInterface')

    def test_gazebo_arg_will_insert_franka_state_interface(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, 'gazebo:=true')
        for transmission in urdf.transmissions:
            if transmission.type == 'franka_hw/FrankaStateInterface':
                self.assertListEqual(
                    ['%s_joint%s' % (arm_id, i) for i in range(1,8)],
                    [joint.name for joint in transmission.joints]
                )
                break
        else:
            # Didn't break out of loop -> no state interface found
            self.fail('No franka_hw/FrankaStateInterface found in URDF')

    def test_gazebo_arg_will_insert_franka_model_interface(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='gazebo:=true')
        for transmission in urdf.transmissions:
            if transmission.type == 'franka_hw/FrankaModelInterface':
                self.assertEqual(
                    arm_id + '_joint1', transmission.joints[0].name,
                    "First joint in the FrankaModelInterface transmission must be the root of the chain, i.e. '$(arm_id)_joint1' not '%s'" % transmission.joints[0].name
                )
                self.assertEqual(
                    arm_id + '_joint8', transmission.joints[1].name,
                    "Second joint in the FrankaModelInterface transmission must be the tip of the chain, i.e. '$(arm_id)_joint8' not '%s'" % transmission.joints[1].name
                )
                break
        else:
            # Didn't break out of loop -> no model interface found
            self.fail('No franka_hw/FrankaModelInterface found in URDF')

    def test_gazebo_arg_and_hand_will_insert_effort_interfaces_for_fingers(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='gazebo:=true hand:=true')
        for joint in [arm_id + '_finger_joint1', arm_id + '_finger_joint2']:
            self.assertJointHasTransmission(urdf, joint, 'hardware_interface/EffortJointInterface')

    def test_setting_gazebo_arg_forces_to_have_no_geometries_inside_sc_links(self):
        urdf = self.xacro(self.file, args='gazebo:=true')
        for name, link in urdf.link_map.items():
            if not name.endswith('_sc'): continue
            self.assertEqual(
                len(link.collisions), 0,
                "Link '%s' is expected to have no <collision> tags defined but has %s" % (name, len(link.collisions))
            )

    def test_setting_gazebo_arg_with_hand_forces_to_have_no_geometries_inside_sc_links(self):
        urdf = self.xacro(self.file, args='gazebo:=true hand:=true')
        for name, link in urdf.link_map.items():
            if not name.endswith('_sc'): continue
            self.assertEqual(
                len(link.collisions), 0,
                "Link '%s' is expected to have no <collision> tags defined but has %s" % (name, len(link.collisions))
            )

    def test_generate_urdf_without_tcp_args_uses_default_10_34_cm_hand_tcp_offset(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='hand:=true')
        joint = urdf.joint_map[arm_id + '_hand_tcp_joint']
        self.assertListEqual(joint.origin.xyz, [0, 0, 0.1034])
        self.assertListEqual(joint.origin.rpy, [0, 0, 0])

    def test_setting_tcp_xyz_arg_moves_the_hand_tcp_link(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='hand:=true tcp_xyz:="1 2 3"')
        joint = urdf.joint_map[arm_id + '_hand_tcp_joint']
        self.assertListEqual(joint.origin.xyz, [1, 2, 3])

    def test_setting_tcp_rpy_arg_rotates_the_hand_tcp_link(self):
        arm_id = self.robot
        urdf = self.xacro(self.file, args='hand:=true tcp_rpy:="3.1415 0.123 42"')
        joint = urdf.joint_map[arm_id + '_hand_tcp_joint']
        self.assertListEqual(joint.origin.rpy, [3.1415, 0.123, 42])

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'URDF', FrankaRobotUrdfTest)
