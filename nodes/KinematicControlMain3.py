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
    # [-2.71018971 -3.50066488  3.77773323  7.16560275 -1.21444    -2.50620125
  # 5.75352924]
    # q_init = [4.14*pi/180,-42.12*pi/180,-31.31*pi/180,-24.30*pi/180,6.25*pi/180,-54.51*pi/180,46.99*pi/180]
    # right_arm_ctrl.init_arm(q_init)


    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # right_arm_ctrl.run()
        right_arm_ctrl.run()    
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
