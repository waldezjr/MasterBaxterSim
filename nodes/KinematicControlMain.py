#!/usr/bin/env python

import rospy

import master_baxter_sim.KinematicControlLoop

def main():
    rospy.init_node('KinematicControlNode')
    right_arm_control_loop = KinematicControlLoop("right") 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
