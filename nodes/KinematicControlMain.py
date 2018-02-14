#!/usr/bin/env python

import rospy
import numpy
from tf.transformations import quaternion_from_matrix

from master_baxter_sim.KinematicControlLoop import KinematicControlLoop
from baxter_pykdl import baxter_kinematics
from master_baxter_sim.Transformations import Transformations
from math import (pi,sin,cos,exp,tan)

def main():
    rospy.init_node('KinematicControlNode')

    # Wait for clock time (when simulating in Gazebo)
    while rospy.get_time() == 0 :
        pass

    kin = baxter_kinematics('right')

    T = Transformations()

    pos = [0.5, 0, 0.2]
    Rd = T.Roty(pi)*T.Rotx(pi/2)
    Rd = numpy.vstack((Rd, [0, 0, 0]))
    Rd = numpy.hstack((Rd, [[0], [0], [0], [1]]))

    initial_pose_q = kin.inverse_kinematics(pos) * pi / 180
    print initial_pose_q

    right_arm_ctrl = KinematicControlLoop("right")
    #initialize arm to q_initial
    right_arm_ctrl.init_arm([0,-pi/6,pi/2,pi/4,-pi/3,pi/4,0])

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
