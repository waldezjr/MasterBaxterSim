from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import subprocess
import sys

class HumanArm(object):
    """docstring for ClassName"""
    def __init__(self, l1, l2, l3, urdf_filename, subject_str):

        self.subject_str = subject_str
        #Get URDF
        command_str = "rosrun xacro xacro {3} l1:={0} l2:={1} l3:={2}".format(l1,l2,l3, urdf_filename)
        print 'command_str', command_str
        try:
            robot_description = subprocess.check_output(
                command_str, shell=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as process_error:
            print process_error.output
            sys.exit(1)

        # print self.robot_description

        self.robot_urdf = URDF.from_xml_string(robot_description)
        self.kdl_kin = KDLKinematics(self.robot_urdf, "base_link", "EEF")

    def get_jacobian(self,joints):

        #check if joints is a numpy vector with 7 elements, otherwise it should throw an exception

        #use joints to calculate jacobian
        J = self.kdl_kin.jacobian(joints)
        Jpos = J[:3,:]

        return Jpos

    def get_manipulability(self, joints):

        J = self.get_jacobian(joints)
        manipulability = np.sqrt(np.linalg.det(J*J.transpose()))

        return manipulability


