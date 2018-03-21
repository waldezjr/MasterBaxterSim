#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import quaternion_from_matrix

from master_baxter_sim.KinematicControlLoop import KinematicControlLoop
from baxter_pykdl import baxter_kinematics
from master_baxter_sim.Transformations import Transformations
from math import (pi,sin,cos,exp,tan)

def circular_traj(t,x_c,tSim, mirror=1):
    
    omega = 2*pi/tSim
    r = 0.05 #radius 
    if t<tSim:
        x_ref = np.matrix([x_c[0]+r*cos(omega*t),x_c[1]+mirror*r*sin(omega*t),x_c[2]])
        x_ref_dot = np.matrix([-r*omega*sin(omega*t),mirror*r*omega*cos(omega*t),0.0])
        x_ref_dot_dot = np.matrix([-r*omega*omega*cos(omega*t),-mirror*r*omega*omega*sin(omega*t),0.0])
    else:
        x_ref = np.matrix([x_c[0]+r,x_c[1],x_c[2]])
        x_ref_dot = np.matrix([0.0,0.0,0.0])
        x_ref_dot_dot = np.matrix([0.0,0.0,0.0])
    return x_ref, x_ref_dot,x_ref_dot_dot

def main():
    rospy.init_node('KinematicControlNode')

    # Wait for clock time (when simulating in Gazebo)
    while rospy.get_time() == 0 :
        pass

    right_arm_ctrl = KinematicControlLoop("right")
    left_arm_ctrl = KinematicControlLoop("left")

    q_untuck_right = [0.08,-1.0,1.19,1.94,-0.67,1.03,0.50]
    q_untuck_left = [-0.08,-1.0,-1.19,1.94,0.67,1.03,-0.50]


    t = 0
    t_ros_cur = -1.0
    x_c_right = [0.7,-0.25,0.15]
    x_c_left = [0.7,0.25,0.15]

    #Position : x, y, z ---> vec
    # x_ref_right = np.matrix([0.65,-0.20,0.10])
    # x_ref_left = np.matrix([0.65,0.20,0.10])

    # x_ref_dot_right = np.matrix([0,0,0])
    #Orientation : x, y, z, w ---> vec, scalar
    orient_ref = np.matrix([0.0,1.0,0.0,0.0])

    right_arm_ctrl.init_arm(q_untuck_right)
    left_arm_ctrl.init_arm(q_untuck_left)

    rate = rospy.Rate(20) 

    while not rospy.is_shutdown():

        if t_ros_cur == -1.0:
            t_ros_cur = rospy.get_time()
            t_ros_old = t_ros_cur
        else:
            t_ros_old = t_ros_cur
            t_ros_cur = rospy.get_time()

        deltaT = t_ros_cur - t_ros_old
        t = t+deltaT

        print t

        x_ref_right,x_ref_dot_right,x_ref_dot_dot_right = circular_traj(t,x_c_right,30,-1)
        x_ref_left,x_ref_dot_left,x_ref_dot_dot_left = circular_traj(t,x_c_left,30)


        right_arm_ctrl.run(x_ref_right,x_ref_dot_right,orient_ref)
        left_arm_ctrl.run(x_ref_left,x_ref_dot_left,orient_ref)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
