#!/usr/bin/env python

import rospy
import numpy as np
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

    q_untuck = [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]

    #Position : x, y, z ---> vec
    x_ref = np.matrix([0.60,-0.20,0.10])
    x_ref_dot = np.matrix([0,0,0])
    #Orientation : x, y, z, w ---> vec, scalar
    orient_ref = np.matrix([0.0,1.0,0.0,0.0])

    right_arm_ctrl.init_arm(q_untuck) 

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        right_arm_ctrl.run(x_ref,x_ref_dot,orient_ref,False)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
