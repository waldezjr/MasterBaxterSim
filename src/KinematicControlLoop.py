import rospy
import numpy

from baxter_pykdl import baxter_kinematics
import baxter_interface
import baxter_external_devices

import time
import sys

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)

from sensor_msgs.msg import JointState

from math import (pi, sin, cos, exp)
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, euler_from_quaternion

class KinematicControlLoop:
	def __init__(self,limb_name):

		#Attributes Initialization
		
		#Name of the limb self is controlling
		self.limb_name = limb_name
		
		#joint limits (same for both limbs)
		self.joints_min_limit = [-1.69, -1.36, -3.04, 0.05, -3.038, -1.57, -3.05]
		self.joints_max_limit = [1.56, 1.04, 3.04, 2.58, 3.057, 2.09, 3.05]
		
		#baxter interface
		self.limb = baxter_interface.Limb(self.limb_name)
        self.limb.set_command_timeout(120.0)  #Timeout for control
        self.limb.set_joint_position_speed(0.5) #max velocity for position control

		self.kin = baxter_kinematics(self.limb_name)
		
		#Subscriber to baxter's EEF pose
		rospy.Subscriber('/robot/limb/' + self.limb_name + '/endpoint_state',
			EndpointState, self.endeffector_callback)

		#Subscriber for baxter's joints
        rospy.Subscriber('/robot/joint_states',JointState,
			self.joint_states_callback)

		#Publisher for baxter's JointCommand
		self.pub_joint_cmd = rospy.Publisher('/robot/limb/' + self.limb_name 
			+'/joint_command', JointCommand)

		#Command Message (POSITION_MODE)
		self.command_msg.mode = JointCommand.POSITION_MODE

		#Kinematic Controller Parameters
		self.K = 1.0
		self.K_orient = 1.0
		self.K_jlimit = 1.0

		#Pose Trajectory Reference
		#Position
			#self.x_ref
		#Velocity
			#self.x_dot_ref
		#Acceleration
			#self.x_dot_dot_ref

		#Orientation (hardcoded to be FIXED)
			#self.R_ref
		

	#Callback Methods
    def joint_states_callback(self, data):

       
		angles = data.position

		angles_right = angles[9:16]
		angles_left = angles[2:9]

		#Angles come from 'data' in a different order than the physical one
		#So it has to be ordered before assigning it to the object
		
		if (self.limb_name == "right"):
			self.actual_angles = [angles_right[2], angles_right[3], angles_right[0],
				angles_right[1], angles_right[4],
				angles_right[5], angles_right[6]]

		if (self.limb_name == "left"):
			self.actual_angles = [angles_left[2], angles_left[3], angles_left[0],
				angles_left[1], angles_left[4],
				angles_left[5], angles_left[6]]

	def endeffector_callback(self, data):

		x = data.pose.position.x
		y = data.pose.position.y
		z = data.pose.position.z

		a = data.pose.orientation.x
		b = data.pose.orientation.y
		c = data.pose.orientation.z
		d = data.pose.orientation.w

		#Ordering 'data' before assigning it
		self.end_effector_position = [x, y, z]
		self.end_effector_orient = [d, a, b, c]

		qx = a
		qy = b
		qz = c
		qw = d

        n = numpy.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) * 1.0

        qx = qx/n
        qy = qy/n
        qz = qz/n
        qw = qw/n

		self.end_effector_orient_matrix = numpy.matrix(
			[[(1.0 - 2.0*qy*qy - 2.0*qz*qz),(2.0*qx*qy - 2.0*qz*qw) , (2.0*qx*qz + 2.0*qy*qw)],
			[(2.0*qx*qy + 2.0*qz*qw), (1.0 - 2.0*qx*qx - 2.0*qz*qz), (2.0*qy*qz - 2.0*qx*qw)],
			[(2.0*qx*qz - 2.0*qy*qw), (2.0*qy*qz + 2.0*qx*qw), (1.0 - 2.0*qx*qx - 2.0*qy*qy)]])

	#Execute one control step
	def run(self):

	


