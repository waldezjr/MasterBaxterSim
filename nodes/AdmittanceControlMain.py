#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import quaternion_from_matrix

from master_baxter_sim.KinematicControlLoop import KinematicControlLoop
from master_baxter_sim.AdmittanceControlLoop import AdmittanceControlLoop
from master_baxter_sim.Transformations import Transformations
from master_baxter_sim.msg import BaxterArmLog

from baxter_pykdl import baxter_kinematics
from math import (pi,sin,cos,exp,tan,sqrt)
import rosbag
import time

def circular_traj(t,x_c,tSim, mirror=1):
    # 'mirror' will mirror the trajectory with respect to the x_base axis
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

def human_traj(t,x_c,tSim):
    t_delta = tSim/8
    r = 0.05 #radius

    x_d = np.matrix([0.0,0.0,0.0])
    p1 = circular_traj(t_delta,x_c,tSim)
    p2 = np.matrix([x_c[0], x_c[1] + 3*r/2, x_c[2]])
    p3 = circular_traj(t_delta,x_c,3*tSim)
    p4 = circular_traj(t_delta,x_c,5*tSim)

    if t < t_delta:
        x_d = circular_traj(t,x_c,tSim)
    elif t >= t_delta and t < 2*t_delta:
        x_d = (t-t_delta) * (p2-p1)/1.25 + p1;
    elif t >= 2*t_delta and t < 3*t_delta:
        x_d = (t-2*t_delta) * (p3-p2)/1.25 + p2;
    elif t >= 3*t_delta and t < 5*t_delta:
        x_d = (t-3*t_delta) * (p4-p3)/2.5 + p3;
    elif t >= 5*t_delta and t < tSim:
        x_d = circular_traj(t,x_c,tSim)
    elif t>= tSim:
        x_d = circular_traj(0,x_c,tSim)

    return x_d

def main(bag):
    rospy.init_node('AdmittanceControlNode')

    # Wait for clock time (when simulating in Gazebo)
    while rospy.get_time() == 0 :
        pass

    # right_arm_ctrl = KinematicControlLoop("right")
    left_arm_ctrl = KinematicControlLoop("left")

    # initial arm configurations
    q_untuck_left = [-0.08,-1.0,-1.19,1.94,0.67,1.03,-0.50]

    q_initial = [-0.2662, -0.7219, -0.9775, 1.2290, 0.6837, 1.3972, -0.3970]

    # Initialize arm
    left_arm_ctrl.init_arm(q_initial)
    left_arm_adm = AdmittanceControlLoop("left")

    t = 0
    t_ros_cur = -1.0
    # center both trajectories
    x_c_left = [0.7,0.25,0.15]

    # x_ref_dot_right = np.matrix([0,0,0])
    #Orientation: x, y, z, w ---> vec, scalar
    orient_ref = np.matrix([0.0,1.0,0.0,0.0])


    # set loop frequency (Hz)
    rate = rospy.Rate(20) 

    # setup rosbag to save data
    armLog = BaxterArmLog()

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
        # get reference trajectories
        x_r_left,x_r_dot_left,x_r_dot_dot_left = circular_traj(t,x_c_left,30)
        x_h = human_traj(t,x_c_left,30)

        # step through admittance controller
         # def run(self, x_r, x_r_dot,x_r_dot_dot,x_current_dot,x_h):
        left_arm_adm.run(x_r_left,x_r_dot_left,x_r_dot_dot_left,left_arm_ctrl.x_dot,x_h)

        # step through kinematic controller
        left_arm_ctrl.run(np.transpose(left_arm_adm.x_ref),np.transpose(left_arm_adm.x_ref_dot),orient_ref)

        # save data in bag
        
        pos, ori = left_arm_ctrl.get_pose_arm()
        armLog.robot_error_pos = sqrt((np.transpose(left_arm_ctrl.pos_error)*left_arm_ctrl.pos_error)[0,0])
        armLog.robot_error_orient = sqrt((np.transpose(left_arm_ctrl.orient_error)*left_arm_ctrl.orient_error)[0,0])
        armLog.EEF_pos.x = pos[0]
        armLog.EEF_pos.y = pos[1]
        armLog.EEF_pos.z = pos[2]
        armLog.f_ext = left_arm_ctrl.force_measured
        bag.write('left_arm_log',armLog)


        rate.sleep()

if __name__ == '__main__':
    try:
        time_now=time.strftime("%H:%M:%S")
        date=time.strftime("%d-%m")
        bag = rosbag.Bag('Kinematic_Control_Bag_'+ date +'_'+ time_now  + '.bag', 'w')
        main(bag)
    except rospy.ROSInterruptException:
        pass
    finally:
        bag.close()
