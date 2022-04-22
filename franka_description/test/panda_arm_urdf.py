import sys
import subprocess
from unittest import TestCase
from urdf_parser_py.urdf import URDF, Mesh, Cylinder, Sphere

PKG = 'franka_description'

def xacro(args=''):
    return URDF.from_xml_string(subprocess.check_output(
        'xacro $(rospack find %s)/robots/panda_arm.urdf.xacro %s' % (PKG, args),
        shell=True)
    )

class TestPandaArmURDF(TestCase):

    def test_generate_urdf_without_xacro_args_contains_link0_up_to_link8(self):
        urdf = xacro()
        for i in range(0, 9):
            link = 'panda_link%s' % i
            self.assertTrue(link in urdf.link_map, 'Link "%s" not found in URDF' % link)

    def test_generate_urdf_without_xacro_args_contains_joint1_up_to_joint8(self):
        urdf = xacro()
        for i in range(1, 9):
            joint = 'panda_joint%s' % i
            self.assertTrue(joint in urdf.joint_map, 'Joint "%s" not found in URDF' % joint)

    def test_generate_urdf_without_xacro_args_dont_use_gripper(self):
        urdf = xacro()
        self.assertFalse(
            list(filter(lambda link: 'hand' in link, urdf.link_map.keys())),
            'Found one or more links containing "hand", probably URDF contains a franka_gripper'
        )
        self.assertFalse(
            list(filter(lambda joint: 'hand' in joint, urdf.joint_map.keys())),
            'Found one or more joints containing "hand", probably URDF contains a franka_gripper'
        )

    def test_generate_urdf_without_xacro_args_uses_fine_collision_models(self):
        urdf = xacro()
        for i in range(0,8):
            link = urdf.link_map['panda_link%s' % i]
            self.assertGreaterEqual(len(link.collisions), 1, "Link '%s' does not have any collision meshes assigned to it" % link.name)
            geometry = link.collisions[0].geometry
            self.assertIsInstance(
                geometry, Mesh,
                "Link '%s' is expected to have mesh geometries in <collision> not '%s'" % (link.name, geometry.__class__.__name__)
            )

    def test_generate_urdf_without_xacro_args_uses_coarse_collision_models_for_sc_links(self):
        urdf = xacro()
        for name in urdf.link_map:
            if not name.endswith('_sc'): continue
            link = urdf.link_map[name]
            geometries = [collision.geometry for collision in link.collisions]
            for geometry in geometries:
                self.assertIsInstance(
                    geometry, (Cylinder, Sphere),
                    "Link '%s' is expected to define only a capsule <collision> geometry (made from Cylinders and Spheres, not '%s')" % (name, geometry.__class__.__name__)
                )

    def test_custom_arm_id_renames_links(self):
        arm_id = 'foo'
        urdf = xacro('arm_id:=%s' % arm_id)
        for link in urdf.link_map.keys():
            self.assertIn(arm_id, link)

    def test_custom_arm_id_renames_joints(self):
        arm_id = 'foo'
        urdf = xacro('arm_id:=%s' % arm_id)
        for joint in urdf.joint_map.keys():
            self.assertIn(arm_id, joint)

    def test_generate_urdf_with_hand_puts_franka_gripper_into_urdf(self):
        arm_id = 'panda'
        urdf = xacro('hand:=true')
        for name in ['hand', 'hand_tcp', 'leftfinger', 'rightfinger']:
            link = '%s_%s' % (arm_id, name)
            self.assertIn(link, urdf.link_map)

        for name in ['hand_joint', 'hand_tcp_joint', 'finger_joint1', 'finger_joint2']:
            joint = '%s_%s' % (arm_id, name)
            self.assertIn(joint, urdf.joint_map)

    def test_custom_arm_id_with_hand_renames_hand_joints_and_links(self):
        arm_id = 'foo'
        urdf = xacro('arm_id:=%s hand:=true' % arm_id)
        for name in ['hand', 'hand_tcp', 'leftfinger', 'rightfinger']:
            link = '%s_%s' % (arm_id, name)
            self.assertIn(link, urdf.link_map)

        for name in ['hand_joint', 'hand_tcp_joint', 'finger_joint1', 'finger_joint2']:
            joint = '%s_%s' % (arm_id, name)
            self.assertIn(joint, urdf.joint_map)

    def test_gazebo_arg_will_add_top_level_world_link(self):
        urdf = xacro('gazebo:=true')

        # Check if the world link exists
        self.assertEqual('world', urdf.get_root())

        # Check if robot is directly connected to the world link
        self.assertListEqual(['world'], [joint.parent for joint in urdf.joints if joint.child == 'panda_link0'])

    def test_gazebo_arg_will_insert_gazebo_ros_control_plugin(self):
        urdf = xacro('gazebo:=true')

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


    def assert_transmissions_with(self, urdf, joints, transmission_type):
        for name in joints:
            for transmission in urdf.transmissions:
                joint = transmission.joints[0]
                interface = 'hardware_interface/' + transmission_type
                if joint.name == name and joint.hardwareInterfaces[0] == interface:
                    # Successfully found matching transmission, break out of loop
                    break
            else:
                # Transmission Loop not broken -> no suitable transmission for joint found
                self.fail('No suitable %s transmission tag for "%s" found in URDF' % (transmission_type, name))

    def test_gazebo_arg_will_insert_position_interfaces(self):
        urdf = xacro('gazebo:=true')
        joints = ['panda_joint%s' % i for i in range(1,8)]
        self.assert_transmissions_with(urdf, joints, 'PositionJointInterface')

    def test_gazebo_arg_will_insert_velocity_interfaces(self):
        urdf = xacro('gazebo:=true')
        joints = ['panda_joint%s' % i for i in range(1,8)]
        self.assert_transmissions_with(urdf, joints, 'VelocityJointInterface')

    def test_gazebo_arg_will_insert_effort_interfaces(self):
        urdf = xacro('gazebo:=true')
        joints = ['panda_joint%s' % i for i in range(1,8)]
        self.assert_transmissions_with(urdf, joints, 'EffortJointInterface')

    def test_gazebo_arg_will_insert_franka_state_interface(self):
        urdf = xacro('gazebo:=true')
        for transmission in urdf.transmissions:
            if transmission.type == 'franka_hw/FrankaStateInterface':
                self.assertListEqual(
                    ['panda_joint%s' % i for i in range(1,8)],
                    [joint.name for joint in transmission.joints]
                )
                break
        else:
            # Didn't break out of loop -> no state interface found
            self.fail('No franka_hw/FrankaStateInterface found in URDF')

    def test_gazebo_arg_will_insert_franka_model_interface(self):
        urdf = xacro('gazebo:=true')
        for transmission in urdf.transmissions:
            if transmission.type == 'franka_hw/FrankaModelInterface':
                self.assertEqual(
                    'panda_joint1', transmission.joints[0].name,
                    "First joint in the FrankaModelInterface transmission must be the root of the chain, i.e. '$(arm_id)_joint1' not '%s'" % transmission.joints[0].name
                )
                self.assertEqual(
                    'panda_joint8', transmission.joints[1].name,
                    "Second joint in the FrankaModelInterface transmission must be the tip of the chain, i.e. '$(arm_id)_joint8' not '%s'" % transmission.joints[1].name
                )
                break
        else:
            # Didn't break out of loop -> no model interface found
            self.fail('No franka_hw/FrankaModelInterface found in URDF')

    def test_gazebo_arg_and_hand_will_insert_effort_interfaces_for_fingers(self):
        urdf = xacro('gazebo:=true hand:=true')
        joints = ['panda_finger_joint1', 'panda_finger_joint2']
        self.assert_transmissions_with(urdf, joints, 'EffortJointInterface')

    def test_setting_gazebo_arg_forces_to_have_no_geometries_inside_sc_links(self):
        urdf = xacro('gazebo:=true')
        for name in urdf.link_map:
            if not name.endswith('_sc'): continue
            link = urdf.link_map[name]
            self.assertEqual(
                len(link.collisions), 0,
                "Link '%s' is expected to have no <collision> tags defined but has %s" % (name, len(link.collisions))
            )


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'URDF', TestPandaArmURDF)
