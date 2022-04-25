import sys
import subprocess
from unittest import TestCase
from urdf_parser_py.urdf import URDF, Mesh, Cylinder, Sphere

PKG = 'franka_description'

def xacro(args=''):
    return URDF.from_xml_string(subprocess.check_output(
        'xacro $(rospack find %s)/robots/dual_panda_example.urdf.xacro %s' % (PKG, args),
        shell=True)
    )

class TestPandaArmExampleURDF(TestCase):

    def test_generate_urdf_without_args_is_possible(self):
        try:
            urdf = xacro()
        except subprocess.CalledProcessError:
            self.fail('Could not generate URDF "hand.urdf.xacro", probably syntax error')

    def test_custom_arm_id_1_renames_links(self):
        arm_id = 'foo'
        urdf = xacro('arm_id_1:=%s' % arm_id)
        for link in urdf.link_map.keys():
            self.assertIn(arm_id, link)

    def test_custom_arm_id_1_renames_joints(self):
        arm_id = 'foo'
        urdf = xacro('arm_id_1:=%s' % arm_id)
        for joint in urdf.joint_map.keys():
            self.assertIn(arm_id, joint)

    def test_custom_arm_id_2_renames_links(self):
        arm_id = 'foo'
        urdf = xacro('arm_id_2:=%s' % arm_id)
        for link in urdf.link_map.keys():
            self.assertIn(arm_id, link)

    def test_custom_arm_id_2_renames_joints(self):
        arm_id = 'foo'
        urdf = xacro('arm_id_2:=%s' % arm_id)
        for joint in urdf.joint_map.keys():
            self.assertIn(arm_id, joint)




if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'dual_panda_example.urdf.xacro', TestPandaArmExampleURDF)
