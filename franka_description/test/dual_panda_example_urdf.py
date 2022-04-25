import sys
import subprocess
from .urdf_test_case import UrdfTestCase, PKG
from urdf_parser_py.urdf import URDF, Mesh, Cylinder, Sphere

file = 'dual_panda_example.urdf.xacro'

class TestPandaArmExampleURDF(UrdfTestCase):

    def test_generate_urdf_without_args_is_possible(self):
        urdf = self.xacro(file) # does not throw

    def test_custom_arm_id_1_renames_links_0_up_to_8(self):
        arm_id = 'foo'
        urdf = self.xacro(file, args='arm_id_1:=%s' % arm_id)
        for link in ['%s_link%s' % (arm_id, i) for i in range(0, 9)]:
            self.assertContainsLink(urdf, link)

    def test_custom_arm_id_1_renames_joints_1_up_to_8(self):
        arm_id = 'foo'
        urdf = self.xacro(file, args='arm_id_1:=%s' % arm_id)
        for joint in ['%s_joint%s' % (arm_id, i) for i in range(1, 9)]:
            self.assertContainsJoint(urdf, joint)

    def test_custom_arm_id_2_renames_links_0_up_to_8(self):
        arm_id = 'foo'
        urdf = self.xacro(file, args='arm_id_2:=%s' % arm_id)
        for link in ['%s_link%s' % (arm_id, i) for i in range(0, 9)]:
            self.assertContainsLink(urdf, link)

    def test_custom_arm_id_2_renames_joints(self):
        arm_id = 'foo'
        urdf = self.xacro(file, args='arm_id_2:=%s' % arm_id)
        for joint in ['%s_joint%s' % (arm_id, i) for i in range(1, 9)]:
            self.assertContainsJoint(urdf, joint)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'dual_panda_example.urdf.xacro', TestPandaArmExampleURDF)
