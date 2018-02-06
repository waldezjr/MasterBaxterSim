#!/usr/bin/env python

import rospy
import numpy
#from baxter_pykdl import baxter_kinematics
import baxter_interface

def printCurJointPos():
	print("testing")
	rospy.loginfo(" ROS")

def main():

	rospy.init_node('jointPositionPublisherExample', anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		#hello_str = "hello world %s" % rospy.get_time()
		#rospy.loginfo(hello_str)
		#pub.publish(hello_str)
		printCurJointPos()
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
