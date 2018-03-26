#!/usr/bin/env python

import rospy
import numpy
from baxter_pykdl import baxter_kinematics
import baxter_interface
from math import pi

from baxter_core_msgs.msg import (
    JointCommand,
    SEAJointState
)

def printCurJointPos(limb):
    #print("testing")
    #rospy.loginfo(" ROS")
    print(limb.joint_angles())
    #print(limb.joint_names())

def force_sensor_callback( data):


    measured_torques = numpy.matrix(data.actual_effort)
    gravity_torques = numpy.matrix(data.gravity_model_effort)

    print 'measured torques\n', measured_torques
    print 'gravity_torques\n', gravity_torques
    # print 'external_torques\n', measured_torques - gravity_torques

    # J = numpy.matrix(baxter_kinematics('left').jacobian())
    # Jp = J[0:3,:]
    # JpInv = numpy.linalg.pinv(Jp)

    # external_torques = (measured_torques-gravity_torques)*JpInv*pi/180

    # print '\nExternal Forces: \n', external_torques


def main():

    rospy.init_node('jointPositionPublisherExample', anonymous=True)
    pub_joint_cmd = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)

    # rospy.Subscriber('/robot/limb/' + 'left' +'/gravity_compensation_torques', SEAJointState, force_sensor_callback)

    limb = baxter_interface.Limb("left")
    limb.set_joint_position_speed(0.5)
    limb.set_command_timeout(120.0)

    cmdMsg = JointCommand()
    cmdMsg.mode = JointCommand.POSITION_MODE

#       q untucked, considered to be the initial q: 'left_w0': 0.669996718047412, 'left_w1': 1.030008750129423, 'left_w2': -0.4999996908801263, 'left_e0': -1.1899720096239292, 'left_e1': 1.940029476507764, 'left_s0': -0.08000000085054282, 'left_s1': -0.9999845808672081

    q = limb.joint_angles() #joint configuration
    q['left_w2'] = 3
    cmdMsg.command = q.values()
    cmdMsg.names = q.keys()

    print(cmdMsg)
    pub_joint_cmd.publish(cmdMsg)

    #positions = dict(zip(self.limb.joint_names(), theta))

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        printCurJointPos(limb)
        pub_joint_cmd.publish(cmdMsg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
