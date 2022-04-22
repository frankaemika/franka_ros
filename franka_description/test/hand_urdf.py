import sys
import subprocess
from unittest import TestCase
from urdf_parser_py.urdf import URDF, Mesh, Cylinder, Sphere

PKG = 'franka_description'

def xacro(args=''):
    return URDF.from_xml_string(subprocess.check_output(
        'xacro $(rospack find %s)/robots/hand.urdf.xacro %s' % (PKG, args),
        shell=True)
    )

class TestHandURDF(TestCase):

    def test_generate_urdf_without_args_is_possible(self):
        try:
            urdf = xacro()
        except subprocess.CalledProcessError:
            self.fail('Could not generate URDF "hand.urdf.xacro", probably syntax error')



if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'hand.urdf.xacro', TestHandURDF)
