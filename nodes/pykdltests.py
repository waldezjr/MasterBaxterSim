#!/usr/bin/env python
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import subprocess
import sys
from math import pi

# 29.6    23.9    7.7
l1 = 29.6 / 100#upper arm length
l2 = 23.9 / 100#forearm length
l3 = 7.7 / 100 #hand (fist) length
command_str = "rosrun xacro xacro /home/waldezjr/catkin_ws/src/master_baxter_sim/robots/human_right_arm.xacro l1:={0} l2:={1} l3:={2}".format(l1,l2,l3)
print 'command_str', command_str

#run xacro and get urdf as string
try:
    robot_description = subprocess.check_output(
        command_str, shell=True, stderr=subprocess.STDOUT)
except subprocess.CalledProcessError as process_error:
    print process_error.output
    sys.exit(1)

# print robot_description

robot = URDF.from_xml_string(robot_description)
kdl_kin = KDLKinematics(robot, "base_link", "EEF")
q = kdl_kin.random_joint_angles()
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics
if q_ik is not None:
    pose_sol = kdl_kin.forward(q_ik) # should equal pose


q = np.array([60.26794860,-16.61584885,47.13758311,58.54795884,131.58005532,26.83723893,0.99987375]) * pi / 180

J = kdl_kin.jacobian(q)
print 'q:', q
print 'q_ik:', q_ik
print 'pose:', pose
if q_ik is not None:
    print 'pose_sol:', pose_sol
print 'J:', J

Jpos = J[:3,:]

print 'Jpos',Jpos
# J is a numpy matrix

manipulability = np.sqrt(np.linalg.det(Jpos*Jpos.transpose()))

print 'manipulability', manipulability