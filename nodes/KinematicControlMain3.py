#!/usr/bin/env python

import rospy
import numpy
from tf.transformations import quaternion_from_matrix

from master_baxter_sim.KinematicControlLoop3 import KinematicControlLoop3
from baxter_pykdl import baxter_kinematics
from master_baxter_sim.Transformations import Transformations
from math import (pi,sin,cos,exp,tan)

def main():
    rospy.init_node('KinematicControlNode')

    # Wait for clock time (when simulating in Gazebo)
    while rospy.get_time() == 0 :
        pass

    kin = baxter_kinematics('right')
    right_arm_ctrl = KinematicControlLoop3("right")

    right_arm_ctrl.init_arm([0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50])
    # right_arm_ctrl.run()   

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        right_arm_ctrl.run()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
