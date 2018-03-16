import rospy
import numpy as np
from baxter_pykdl import baxter_kinematics
import baxter_interface
import baxter_external_devices

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

from master_baxter_sim.Transformations import Transformations

import time

def rotxs(angulo):
  Rx = np.matrix([[1.0, 0.0, 0.0],[0.0, cos(angulo), -sin(angulo)],[0.0, sin(angulo), cos(angulo)]])
  return Rx

def rotys(angulo):
  Ry = np.matrix([[cos(angulo), 0.0, sin(angulo)],[0.0, 1.0, 0.0],[-sin(angulo), 0.0, cos(angulo)]])
  return Ry

def rotzs(angulo):
  Rz = np.matrix([[cos(angulo), -sin(angulo), 0.0],[sin(angulo), cos(angulo), 0.0],[0.0, 0.0, 1.0]])
  return Rz


def calcjacobn(t1,t2,t3,t4,t5,t6,t7):
  l1 = 0.270
  l2 = 0.069
  l3 = 0.102
  l4 = 0.262
  l5 = 0.069
  l6 = 0.104
  l7 = 0.271
  l8 = 0.010
  l9 = 0.116
  l10 = 0.230
  R01 = rotzs(t1)
  R02 = R01*rotys(t2)
  R03 = R02*rotxs(t3)
  R04 = R03*rotys(t4)
  R05 = R04*rotxs(t5)
  R06 = R05*rotys(t6)
  R07 = R06*rotxs(t7)
  h1 = np.transpose(np.array(R01*np.matrix([[0.0],[0.0],[1.0]])))
  h2 = np.transpose(np.array(R02*np.matrix([[0.0],[1.0],[0.0]])))
  h3 = np.transpose(np.array(R03*np.matrix([[1.0],[0.0],[0.0]])))
  h4 = np.transpose(np.array(R04*np.matrix([[0.0],[1.0],[0.0]])))
  h5 = np.transpose(np.array(R05*np.matrix([[1.0],[0.0],[0.0]])))
  h6 = np.transpose(np.array(R06*np.matrix([[0.0],[1.0],[0.0]])))
  h7 = np.transpose(np.array(R07*np.matrix([[1.0],[0.0],[0.0]])))
  p1e = R01*np.matrix([[l2],[0.0],[l1]]) + R02*np.matrix([[l3],[0.0],[0.0]]) + R03*np.matrix([[l4],[0.0],[-l5]]) + R04*np.matrix([[l6],[0.0],[0.0]]) + R05*np.matrix([[l7],[0.0],[-l8]]) + R06*np.matrix([[l9],[0.0],[0.0]]) + R07*np.matrix([[l10],[0.0],[0.0]])
  p2e = R02*np.matrix([[l3],[0.0],[0.0]]) + R03*np.matrix([[l4],[0.0],[-l5]]) + R04*np.matrix([[l6],[0.0],[0.0]]) + R05*np.matrix([[l7],[0.0],[-l8]]) + R06*np.matrix([[l9],[0.0],[0.0]]) + R07*np.matrix([[l10],[0.0],[0.0]])
  p3e = R03*np.matrix([[l4],[0.0],[-l5]]) + R04*np.matrix([[l6],[0.0],[0.0]]) + R05*np.matrix([[l7],[0.0],[-l8]]) + R06*np.matrix([[l9],[0.0],[0.0]]) + R07*np.matrix([[l10],[0.0],[0.0]])
  p4e = R04*np.matrix([[l6],[0.0],[0.0]]) + R05*np.matrix([[l7],[0.0],[-l8]]) + R06*np.matrix([[l9],[0.0],[0.0]]) + R07*np.matrix([[l10],[0.0],[0.0]])
  p5e = R05*np.matrix([[l7],[0.0],[-l8]]) + R06*np.matrix([[l9],[0.0],[0.0]]) + R07*np.matrix([[l10],[0.0],[0.0]])
  p6e = R06*np.matrix([[l9],[0.0],[0.0]]) + R07*np.matrix([[l10],[0.0],[0.0]])
  p7e = R07*np.matrix([[l10],[0.0],[0.0]])
  p1e = np.transpose(np.array(p1e))
  p2e = np.transpose(np.array(p2e))
  p3e = np.transpose(np.array(p3e))
  p4e = np.transpose(np.array(p4e))
  p5e = np.transpose(np.array(p5e))
  p6e = np.transpose(np.array(p6e))
  p7e = np.transpose(np.array(p7e))
  JE0A = np.hstack((np.matrix(np.transpose(np.cross(h1,p1e))),np.matrix(np.transpose(np.cross(h2,p2e))),np.matrix(np.transpose(np.cross(h3,p3e))),np.matrix(np.transpose(np.cross(h4,p4e))),np.matrix(np.transpose(np.cross(h5,p5e))),np.matrix(np.transpose(np.cross(h6,p6e))),np.matrix(np.transpose(np.cross(h7,p7e)))))
  JE0B = np.hstack((np.matrix(np.transpose(h1)),np.matrix(np.transpose(h2)),np.matrix(np.transpose(h3)),np.matrix(np.transpose(h4)),np.matrix(np.transpose(h5)),np.matrix(np.transpose(h6)),np.matrix(np.transpose(h7))))
  JE0 = np.vstack((JE0A,JE0B))
  A = np.vstack((np.hstack((np.transpose(R07),np.matrix(np.zeros((3,3))))),np.hstack((np.matrix(np.zeros((3,3))),np.transpose(R07)))))
  Jen = A*JE0
  return Jen

class KinematicControlLoop2:
    def __init__(self,limb_name):

        #Attributes Initialization
        
        #Name of the limb self is controlling
        self.limb_name = limb_name
        rospy.loginfo("Initializing Baxter's %s arm kinematic control", self.limb_name)
        rospy.loginfo("Arm Controlled in POSITION MODE")
        
        #joint limits (same for both limbs)
        self.joints_min_limit = [-1.69, -1.36, -3.04, 0.05, -3.038, -1.57, -3.05]
        self.joints_max_limit = [1.56, 1.04, 3.04, 2.58, 3.057, 2.09, 3.05]
        
        #baxter interface
        self.limb = baxter_interface.Limb(self.limb_name)
        self.limb.set_command_timeout(120.0)  #Timeout for control
        self.limb.set_joint_position_speed(0.5) #max velocity for position control

        #As of now, there is a warning coming from baxter_kinematics
        self.kin = baxter_kinematics(self.limb_name)
        
        #Subscriber to baxter's EEF pose
        rospy.Subscriber('/robot/limb/' + self.limb_name + '/endpoint_state',
            EndpointState, self.endeffector_callback)

        #Subscriber for baxter's joints
        rospy.Subscriber('/robot/joint_states',JointState,
            self.joint_states_callback)

        #Publisher for baxter's JointCommand
        self.pub_joint_cmd = rospy.Publisher('/robot/limb/' + self.limb_name 
            +'/joint_command', JointCommand, queue_size=1)

        self.command_msg = JointCommand()
        self.command_msg.mode = JointCommand.POSITION_MODE


        #Kinematic Controller Parameters
        self.K_kin = 10000 * np.eye(3, dtype=float)

        print self.K_kin

        #Current and old time parameter
        self.current_time = -1.0
        self.old_time = 0.0

        #Current Pose

        self.end_effector_position = self.limb.endpoint_pose()['position']
        self.end_effector_orient = self.rotate_list(self.limb.endpoint_pose()['orientation'],1)

        #Pose Trajectory Reference
        #Position
        self.x_ref = np.matrix([0.5,-0.2,0.3])
        #Velocity
        self.x_dot_ref = np.matrix([0,0,0]) 
     
    def rotate_list(self,l, n):
        return l[n:] + l[:n]    

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

        n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) * 1.0

        qx = qx/n
        qy = qy/n
        qz = qz/n
        qw = qw/n


    def init_arm(self, init_q):
        
        #Enable Baxter
        rs = baxter_interface.RobotEnable()
        init_state = rs.state().enabled
        rs.enable()

        j_a = self.limb.joint_angles()
        # j_a['right_s0']=0.0
        j_a['right_s0']= init_q[0]
        j_a['right_s1']= init_q[1]
        j_a['right_e0']= init_q[2]
        j_a['right_e1']= init_q[3]
        j_a['right_w0']= init_q[4]
        j_a['right_w1']= init_q[5]
        j_a['right_w2']= init_q[6]
        self.limb.move_to_joint_positions(j_a)

        # print self.limb.endpoint_pose()

    def get_angles_right_arm(self):
        q_aux = self.limb.joint_angles()
        q = [q_aux['right_s0'], q_aux['right_s1'], q_aux['right_e0'], q_aux['right_e1'], q_aux['right_w0'], q_aux['right_w1'], q_aux['right_w2']]
        return q

    def get_pos_right_arm(self):
        return self.limb.endpoint_pose()['position']

    #Execute one control step
    def run(self):

        #get current time
        if self.current_time == -1.0:
            self.current_time = rospy.get_time()
            # self.current_time = time.time()
            self.old_time = self.current_time
        else:
            self.old_time = self.current_time
            self.current_time = rospy.get_time()
            # self.current_time = time.time()

        #Convert EEF position to np vector
        # x_current = self.end_effector_position
        x_current = self.get_pos_right_arm()
        x_current = np.matrix(x_current)

        print 'x_current', x_current

        self.pos_error = np.transpose(np.add(self.x_ref,-1.0*x_current))

        # print 'pos_error', self.pos_error

        # calcjacobn
        J = np.matrix(self.kin.jacobian())
        # J2 = calcjacobn(self.actual_angles[0],self.actual_angles[1],self.actual_angles[2],self.actual_angles[3],self.actual_angles[4],self.actual_angles[5],self.actual_angles[6])
        Jp = J[0:3,:]
        JpInv = np.linalg.pinv(Jp)

        # print 'JpInv', JpInv
        # print 'J2', J2[0:3,:]

        q_dot = JpInv * (np.transpose(self.x_dot_ref) + self.K_kin*(self.pos_error))

        # print 'joints velocities', q_dot

        deltaT = self.current_time - self.old_time

        # q = deltaT * q_dot + np.transpose(np.matrix(self.actual_angles))
        q = deltaT * q_dot + np.transpose(np.matrix(self.get_angles_right_arm()))

        # print 'joints position cmd', q

        # self.limb.joint_names() # = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

        # q: 'left_w0': 0.669996718047412, 'left_w1': 1.030008750129423, 'left_w2': -0.4999996908801263, 'left_e0': -1.1899720096239292, 'left_e1': 1.940029476507764, 'left_s0': -0.08000000085054282, 'left_s1': -0.9999845808672081

        q_list = np.squeeze(np.asarray(q)).tolist()

        # print 'q list', q_list
        print 'cur joint angles API', self.limb.joint_angles()
        print 'current joint angles', self.actual_angles

        self.command_msg.mode = JointCommand.POSITION_MODE
        self.command_msg.names = self.limb.joint_names()
        self.command_msg.command = q_list

        #Publish joint position command
        self.pub_joint_cmd.publish(self.command_msg)

        
        # saturation = 5.0

        # for k in range(len(theta_dot_vector)):
        #     if (theta_dot_vector[k] > saturation):
        #         theta_dot_vector[k] = saturation
        #     elif (theta_dot_vector[k] < -saturation):
        #         theta_dot_vector[k] = -saturation

        # # if (np.linalg.norm(error_vect) < 0.001):
        # #     theta_dot_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



        # print "theta_dot_vector_final", theta_dot_vector
        # #velocities = dict(zip(self.limb.joint_names(), theta_dot_vector))

        # type(theta_dot_vector)
        # # Integrate q_dot to use it as a q command
        # deltaT = self.current_time - self.old_time
        # pos_cmd_vec = np.add(np.multiply(deltaT, theta_dot_vector) , np.array(self.actual_angles))
        # print ("Current position", self.actual_angles)
        # print ("Commanded position", pos_cmd_vec)

        # joint_positions = dict(zip(self.limb.joint_names(), pos_cmd_vec))

        # self.command_msg.mode = JointCommand.POSITION_MODE
        # self.command_msg.names = joint_positions.keys()
        # self.command_msg.command = joint_positions.values()
        # print ('deltaT', deltaT)

        # #Publish joint position command
        # self.pub_joint_cmd.publish(self.command_msg)

        # velocities = dict(zip(self.limb.joint_names(), theta_dot_vector))

        # self.command_msg.mode = JointCommand.VELOCITY_MODE
        # self.command_msg.names = velocities.keys()
        # self.command_msg.command = velocities.values()

        # #self.rate.sleep()
        # self.pub_joint_cmd.publish(self.command_msg)  ##Send the velocity to the internal controller


    


