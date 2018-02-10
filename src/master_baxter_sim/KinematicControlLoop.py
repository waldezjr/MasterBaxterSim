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

        #Convert EEF position to numpy vector
        x_current = self.end_effector_position
        x_current = numpy.matrix(x_current)

        #Convert orientation matrix to numpy matrix and transpose it
        orient_current = self.end_effector_orient
        orient_current = numpy.matrix(orient_current)
        orient_current = numpy.transpose(orient_current)

        self.pos_error = numpy.transpose(numpy.add(self_x_ref,-1.0*x_current))

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
        #print "theta_dot_vector depois",theta_dot_vector



        #Add saturation to velocity

        saturation = 5.0

        for k in range(len(theta_dot_vector)):
            if (theta_dot_vector[k] > saturation):
                theta_dot_vector[k] = saturation
            elif (theta_dot_vector[k] < -saturation):
                theta_dot_vector[k] = -saturation

        if (numpy.linalg.norm(error_vect) < 0.001):
            theta_dot_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #Populate and send JointCommand msg to Baxter

        #print "theta_dot_vector_final", theta_dot_vector
        velocities = dict(zip(self.limb.joint_names(), theta_dot_vector))

        self.command_msg.mode = JointCommand.VELOCITY_MODE
        self.command_msg.names = velocities.keys()
        self.command_msg.command = velocities.values()

        #self.rate.sleep()
        #self.pub_joint_cmd.publish(self.command_msg)  ##Send the velocity to the internal controller

    


