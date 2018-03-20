#!/usr/bin/env python

import rospy
import numpy as np
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

    right_arm_ctrl = KinematicControlLoop("right")
    left_arm_ctrl = KinematicControlLoop("left")

    q_untuck_right = [0.08,-1.0,1.19,1.94,-0.67,1.03,0.50]
    q_untuck_left = [-0.08,-1.0,-1.19,1.94,0.67,1.03,-0.50]

    #Position : x, y, z ---> vec
    x_ref_right = np.matrix([0.65,-0.20,0.10])
    x_ref_left = np.matrix([0.65,0.20,0.10])

    x_ref_dot = np.matrix([0,0,0])
    #Orientation : x, y, z, w ---> vec, scalar
    orient_ref = np.matrix([0.0,1.0,0.0,0.0])

    right_arm_ctrl.init_arm(q_untuck_right)
    left_arm_ctrl.init_arm(q_untuck_left)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        right_arm_ctrl.run(x_ref_right,x_ref_dot,orient_ref,False)
        left_arm_ctrl.run(x_ref_left,x_ref_dot,orient_ref,False)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
