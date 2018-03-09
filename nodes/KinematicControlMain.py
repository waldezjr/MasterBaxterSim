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
    right_arm_ctrl = KinematicControlLoop("right")

    T = Transformations()

    pos = [0.5, 0, 0.2] 
    
    # Rd = T.Roty(pi)*T.Rotx(pi/2) 
    # Rd = numpy.vstack((Rd, [0, 0, 0])) 
    # Rd = numpy.hstack((Rd, [[0], [0], [0], [1]]))
    # orient = quaternion_from_matrix(Rd)
    # orient = [orient[3], orient[0], orient[1], orient[2]]
    # orient = numpy.matrix(orient)

    # print 'orient', orient
    # rospy.loginfo(orient)
    # initial_pose_q = kin.inverse_kinematics(pos,orient.tolist()) * pi / 180
    # rospy.loginfo(initial_pose_q)
    # print 'initial_pose_q', initial_pose_q

    #initialize arm to q_initial
    # right_arm_ctrl.init_arm([0,-pi/6,pi/2,pi/4,-pi/3,pi/4,0])

    right_arm_ctrl.init_arm([0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50])
    # right_arm_ctrl.x_ref
    # right_arm_ctrl.init_arm(initial_pose_q)

    # right_arm_ctrl.run()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        right_arm_ctrl.run()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
