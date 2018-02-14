#!/usr/bin/env python

import rospy

from master_baxter_sim.KinematicControlLoop import KinematicControlLoop

def main():
    rospy.init_node('KinematicControlNode')

    # Wait for clock time (when simulating in Gazebo)
    while rospy.get_time() == 0 :
        pass

    right_arm_ctrl = KinematicControlLoop("right")
    #initialize arm to q_initial
    right_arm_ctrl.init_arm()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
