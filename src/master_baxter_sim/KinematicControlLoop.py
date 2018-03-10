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

from master_baxter_sim.Transformations import Transformations

class KinematicControlLoop:
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
        self.K = 1.0
        self.K_orient = 1.0
        self.K_jlimit = 10.0

        #Current and old time parameter
        self.current_time = -1.0
        self.old_time = 0.0

        #Current Pose

        self.end_effector_position = self.limb.endpoint_pose()['position']
        self.end_effector_orient = self.rotate_list(self.limb.endpoint_pose()['orientation'],1)

        #Pose Trajectory Reference
        #Position
        self.x_ref = numpy.matrix([0.55,0,0.2])
        #Velocity
        self.x_dot_ref = [0,0,0] 

        #Orientation 
        T = Transformations()

        Rd = T.Rotx(pi/2)*T.Roty(pi) 
        Rd = numpy.vstack((Rd, [0, 0, 0])) 
        Rd = numpy.hstack((Rd, [[0], [0], [0], [1]]))
        orient = quaternion_from_matrix(Rd)
        orient = [orient[3], orient[0], orient[1], orient[2]]

        print orient

        self.orient_ref = numpy.matrix([[sin(3.14/2)],[0],[0],[cos(3.14/2)]]) # x y z w
        print '/n orientation ref:', self.orient_ref
     
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

        n = numpy.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) * 1.0

        qx = qx/n
        qy = qy/n
        qz = qz/n
        qw = qw/n

        self.end_effector_orient_matrix = numpy.matrix(
            [[(1.0 - 2.0*qy*qy - 2.0*qz*qz),(2.0*qx*qy - 2.0*qz*qw) , (2.0*qx*qz + 2.0*qy*qw)],
            [(2.0*qx*qy + 2.0*qz*qw), (1.0 - 2.0*qx*qx - 2.0*qz*qz), (2.0*qy*qz - 2.0*qx*qw)],
            [(2.0*qx*qz - 2.0*qy*qw), (2.0*qy*qz + 2.0*qx*qw), (1.0 - 2.0*qx*qx - 2.0*qy*qy)]])


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

    #Execute one control step
    def run(self):

        #get current time
        if self.current_time == -1.0:
            self.current_time = rospy.get_time()
        else:
            self.old_time = self.current_time
            self.current_time = rospy.get_time()

        #Convert EEF position to numpy vector
        x_current = self.end_effector_position
        x_current = numpy.matrix(x_current)

        #Convert orientation matrix to numpy matrix and transpose it
        orient_current = self.end_effector_orient
        orient_current = numpy.matrix(orient_current)
        orient_current = numpy.transpose(orient_current)

        self.pos_error = numpy.transpose(numpy.add(self.x_ref,-1.0*x_current))

        x_dot = self.K * self.pos_error

        orient_ref = self.orient_ref
        q0d = float(orient_ref[0])
        q0 = float(orient_current[0])
        qvd = [float(orient_ref[1]), float(orient_ref[2]), float(orient_ref[3])]
        qv = [float(orient_current[1]), float(orient_current[2]), float(orient_current[3])]

        cross_result = numpy.cross(qvd, qv)
        cross_result = numpy.matrix(cross_result)

        qvd = numpy.matrix(qvd)
        qv = numpy.matrix(qv)

        #Get orientation error
        orient_error = -q0d * qv + q0 * qvd - cross_result
        self.orient_error = numpy.transpose(orient_error)

        omega = self.K_orient * orient_error
        omega = numpy.transpose(omega)

        ### Calculate Jacobian #######

        # Jacobian
        J = numpy.matrix(self.kin.jacobian())
        J_t = numpy.transpose(J)
        I_6 = numpy.matrix(numpy.identity(6), copy=False)

        # Jacobian Pseudo-Inverse (Moore-Penrose) + DLS
        manip = numpy.linalg.det(J * J_t)
        error_vect = numpy.vstack((self.pos_error, self.orient_error))
        beta = pow(numpy.linalg.norm(error_vect), 2) / (manip * 300)  #numpy.linalg.norm(pos_error)
        J_pseudoinv = J_t * numpy.linalg.inv(J * J_t + pow(beta, 2) * I_6)

        control = numpy.vstack((x_dot, omega))
        #print "control", control
        theta_dot_vector = J_pseudoinv * control
        #print "theta_dot_vector1",theta_dot_vector

        ######################## Velocity Control Input ##########################################

        ######################## Adds the joint limit weighting ##################################

        n = 7

        I = numpy.matrix(numpy.identity(n), copy=False)
        M = I - (J_pseudoinv * J)

        dw_dtheta = []
        theta_mean_vector = []

        for k in range(len(self.joints_min_limit)):
            theta = self.actual_angles[k]
            theta_m = self.joints_min_limit[k]
            theta_M = self.joints_max_limit[k]
            theta_mean = (theta_m + theta_M) / 2
            theta_mean_vector += [theta_mean]

            dw_dtheta += [(theta - theta_mean) / pow((theta_M - theta_m), 2)]
        #dw_dtheta+=[2*(theta-theta_mean)]

        if (len(dw_dtheta) != n):
            print "error in dw_dtheta dimension!!"

        #print "dw_dtheta",dw_dtheta
        dw_dtheta = numpy.matrix(dw_dtheta)
        #dw_dtheta=numpy.transpose(dw_dtheta)

        mi = (self.K_jlimit) * ((-1.0 / n) * dw_dtheta)
        mi = numpy.transpose(mi)

        u_2 = M * mi

        u_2 = numpy.mat(u_2).tolist()
        u_2_vector = []

        for i in u_2:
            u_2_vector += [numpy.float(i[0])]

        #print "u_2_vector",u_2_vector

        theta_dot_vector_temp = numpy.mat(theta_dot_vector).tolist()
        theta_dot_vector = []

        for i in theta_dot_vector_temp:
            theta_dot_vector += [numpy.float(i[0])]

        #print "theta_dot_vector antes",theta_dot_vector


        theta_dot_vector = numpy.add(theta_dot_vector, u_2_vector)
        # print "theta_dot_vector depois",theta_dot_vector



        #Add saturation to velocity

        saturation = 5.0

        for k in range(len(theta_dot_vector)):
            if (theta_dot_vector[k] > saturation):
                theta_dot_vector[k] = saturation
            elif (theta_dot_vector[k] < -saturation):
                theta_dot_vector[k] = -saturation

        if (numpy.linalg.norm(error_vect) < 0.001):
            theta_dot_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # print "theta_dot_vector_final", theta_dot_vector
        #velocities = dict(zip(self.limb.joint_names(), theta_dot_vector))

        #Integrate q_dot to use it as a q command
        deltaT = self.current_time - self.old_time
        pos_cmd_vec = deltaT * (theta_dot_vector) + numpy.array(self.actual_angles)
        # print ("Current position", self.actual_angles)
        # print ("Commanded position", pos_cmd_vec)
        # print ('deltaT', deltaT)

        #self.command_msg.mode = JointCommand.VELOCITY_MODE
        self.command_msg.names = self.limb.joint_names()
        self.command_msg.command = pos_cmd_vec

        #Publish joint position command
        # self.pub_joint_cmd.publish(self.command_msg)

        # velocities = dict(zip(self.limb.joint_names(), theta_dot_vector))

        # self.command_msg.mode = JointCommand.VELOCITY_MODE
        # self.command_msg.names = velocities.keys()
        # self.command_msg.command = velocities.values()

        # #self.rate.sleep()
        # self.pub_joint_cmd.publish(self.command_msg)  ##Send the velocity to the internal controller


    


