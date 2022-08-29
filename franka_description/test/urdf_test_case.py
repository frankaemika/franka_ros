import subprocess

from urdf_parser_py.urdf import URDF
from unittest import TestCase

PKG = 'franka_description'

class UrdfTestCase(TestCase):
    def xacro(self, file, args=''):
        """
        Generates a URDF from a XACRO file. If the URDF is invalid the test is
        automatically failed.
        @param file: The name of the xacro input file within `franka_description/robots/`
        @param args: A optional string of xacro args to append, e.g. ("foo:=1 bar:=true")
        @return: The generated URDF, as urdf_parser_py.urdf.URDF type
        """
        try:
            return URDF.from_xml_string(subprocess.check_output(
                'xacro $(rospack find %s)/robots/%s %s' % (PKG, file, args),
                shell=True)
            )
        except subprocess.CalledProcessError as e:
            self.fail('Could not generate URDF from "%s", probably syntax error: %s' % (file, e.output))

    def assertContainsLink(self, urdf, link):
        """
        Check that the URDF contains a certain link
        @param urdf: The URDF to test
        @param link: The name of the link to find
        @return: True if 'link' is present in 'urdf', False otherwise
        """
        self.assertIn(link, urdf.link_map, 'Link "%s" not found in URDF' % link)

    def assertContainsJoint(self, urdf, joint):
        """
        Check that the URDF contains a certain joint
        @param urdf: The URDF to test
        @param joint: The name of the joint to find
        @return: True if 'joint' is present in 'urdf', False otherwise
        """
        self.assertIn(joint, urdf.joint_map, 'Joint "%s" not found in URDF' % joint)

    def links_with(self, urdf, predicate):
        """
        Iterate all links in the URDF and filter with a predicate
        @param urdf: The URDF with the links to filter
        @param predicate: A function, which gets the link name as argument and which
                          should return true if the link should be in the list or not
        @return: A list of link names matching the predicate
        """
        return list(filter(predicate, urdf.link_map.keys()))

    def joints_with(self, urdf, predicate):
        """
        Iterate all joints in the URDF and filter with a predicate
        @param urdf: The URDF with the joints to filter
        @param predicate: A function, which gets the joint name as argument and which
                          should return true if the joint should be in the list or not
        @return: A list of joint names matching the predicate
        """
        return list(filter(predicate, urdf.joint_map.keys()))

    def collision_geometries(self, urdf, link):
        """
        Find all <collision> geometries of a certain link in a URDF.
        Fail the test if the link does not exist.
        @param urdf: The URDF to search for
        @param link: The name of the link to get its collision geometries for
        @return: A list of urdf_parser_py.urdf.Geometry
        """
        self.assertContainsLink(urdf, link)
        return [ collision.geometry for collision in urdf.link_map[link].collisions ]

    def assertJointBetween(self, urdf, parent, child, type=None):
        """
        Assert that there exists a <joint> between two links
        @param urdf: The URDF in which to look for such a joint
        @param parent: The name of the parent link of the joint to look for
        @param child:  The name of the child link of the joint to look for
        @param type:   None, to search for any joint, or one of ['unknown', 'revolute',
                       'continuous', 'prismatic', 'floating', 'planar', 'fixed'] to check
                       for a specific type only.
        """
        candidates = list(filter(lambda j: j.parent == parent, urdf.joints))
        self.assertTrue(candidates, "Could not find any joint in URDF which parent is '%s'" % parent)

        candidates = list(filter(lambda j: j.child == child, candidates))
        self.assertTrue(candidates, "Could not find any joint in URDF which child is '%s'" % child)

        if type is not None:
            candidates = list(filter(lambda j: j.joint_type == type, candidates))
            self.assertTrue(candidates, "Could not find any joint in URDF from %s -> %s which is %s" % (parent, child, type))

    def assertJointHasTransmission(self, urdf, joint, type):
        """
        Assert that there exists a <transmission> to a joint with a certain type
        @param urdf: The URDF in which to look for the transmission
        @param joint: The name of the joint for which the transmission is for
        @param type: The type of the hw interface, e.g. 'hardware_interface/PositionJointInterface'
        """
        self.assertContainsJoint(urdf, joint)
        for transmission in urdf.transmissions:
            j = transmission.joints[0]
            if j.name == joint and j.hardwareInterfaces[0] == type:
                # Successfully found matching transmission, break out of loop
                break
        else:
            # Transmission Loop not broken -> no suitable transmission for joint found
            self.fail('No suitable "%s" transmission tag for "%s" found in URDF' % (type, joint))
