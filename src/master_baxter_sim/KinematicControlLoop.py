import rospy
import numpy as np
from baxter_pykdl import baxter_kinematics
import baxter_interface
import baxter_external_devices
import scipy.linalg
import tf
import tf2_ros
from geometry_msgs.msg import (
    WrenchStamped,
    Vector3
)

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
    SEAJointState,
)

# from sensor_msgs.msg import JointState

from math import (pi, sin, cos, exp,sqrt)

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

        self.joint_max_speed = 2.0
        
        #baxter interfac
        self.limb = baxter_interface.Limb(self.limb_name)
        self.limb.set_command_timeout(0.1)  #Timeout for control
        self.limb.set_joint_position_speed(self.joint_max_speed) #max velocity for position control

        #As of now, there is a warning coming from baxter_kinematics
        self.kin = baxter_kinematics(self.limb_name)

        #Publisher for baxter's JointCommand
        self.pub_joint_cmd = rospy.Publisher('/robot/limb/' + self.limb_name 
            +'/joint_command', JointCommand, queue_size=1)

        self.command_msg = JointCommand()
        self.command_msg.mode = JointCommand.POSITION_MODE


        #Kinematic Controller Parameters
        self.K_kin_p = 120 * np.eye(3, dtype=float)
        self.K_kin_o = 120 * np.eye(3, dtype=float)

        self.K_kin = scipy.linalg.block_diag(self.K_kin_p,self.K_kin_o)

        # print self.K_kin_p

        print '\n Kinematic controller gain matrix \n', self.K_kin

        #Current and old time parameter
        self.current_time = -1.0
        self.old_time = 0.0

        #Current Pose


        self.kin.print_kdl_chain()
        # print 'ik BAXTER pykdl', self.kin.inverse_kinematics([0.5,-0.2,0.3])

        # print self.get_angles_arm()

        # REMEMBER TO TEST THIS TO BE SURE PYKDL IS USING DEG INSTEAD OF RAD
        # print self.kin.forward_position_kinematics([4.14,-42.12,-31.31,-24.30,6.25,-54.51,46.99])
        self.force_measured = Vector3()
        self.x_dot = np.matrix([0.0,0.0,0.0])

    def rotate_list(self,l, n):
        return l[n:] + l[:n]

    # rotate vector v1 by quaternion q1 
    def qv_mult(self,v1,q1):
        len =sqrt(v1.x**2 + v1.y**2 + v1.z**2 )
        v1 = tf.transformations.unit_vector(np.array([v1.x,v1.y,v1.z]))
        q2 = list(v1)
        q2.append(0.0)
        q1 = np.array([q1.x,q1.y,q1.z,q1.w])
        v1 = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2), 
            tf.transformations.quaternion_conjugate(q1)
        )[:3]
        v1 = v1*len
        return v1

    def init_arm(self, init_q):
        
        #Enable Baxter
        rs = baxter_interface.RobotEnable()
        init_state = rs.state().enabled
        rs.enable()

        j_a = self.limb.joint_angles()

        # print j_a

        if(self.limb_name == 'right'):
            j_a['right_s0']= init_q[0]
            j_a['right_s1']= init_q[1]
            j_a['right_e0']= init_q[2]
            j_a['right_e1']= init_q[3]
            j_a['right_w0']= init_q[4]
            j_a['right_w1']= init_q[5]
            j_a['right_w2']= init_q[6]
        else:
            j_a['left_s0']= init_q[0]
            j_a['left_s1']= init_q[1]
            j_a['left_e0']= init_q[2]
            j_a['left_e1']= init_q[3]
            j_a['left_w0']= init_q[4]
            j_a['left_w1']= init_q[5]
            j_a['left_w2']= init_q[6]
        self.limb.move_to_joint_positions(j_a)
        self.end_effector_position = self.limb.endpoint_pose()['position']
        self.end_effector_orient = self.limb.endpoint_pose()['orientation']

    def get_angles_arm(self):
        q_aux = self.limb.joint_angles()
        if self.limb_name == 'right':
            q = [q_aux['right_s0'], q_aux['right_s1'], q_aux['right_e0'], q_aux['right_e1'], q_aux['right_w0'], q_aux['right_w1'], q_aux['right_w2']]
        else:
            q = [q_aux['left_s0'], q_aux['left_s1'], q_aux['left_e0'], q_aux['left_e1'], q_aux['left_w0'], q_aux['left_w1'], q_aux['left_w2']]
        return q

    def get_pose_arm(self):
        pos = self.limb.endpoint_pose()['position']
        # pos: x,y,z ---> vec
        orientation = self.limb.endpoint_pose()['orientation']
        # orient: x,y,z,w ---> vec,scalar
        return pos,orientation

    def pos_control(self,deltaT):
        
        x_current, x_orient = self.get_pose_arm()

        #Convert EEF position to np vector
        x_current = np.matrix(x_current)
        x_orient = np.matrix(x_orient)
        # print 'x_current', x_current
        # print 'x_orient', x_orient

        self.pos_error = np.transpose(np.add(self.x_ref,-1.0*x_current))
        if self.verbose:
            print 'pos_error', self.pos_error        

        J = np.matrix(self.kin.jacobian())
        # print 'J', J

        Jp = J[0:3,:]
        # print 'Jp', Jp        

        JpInv = np.linalg.pinv(Jp)
        # print 'JpInv', JpInv    

        q_dot = JpInv * (np.transpose(self.x_dot_ref) + self.K_kin_p*(self.pos_error))
        q_dot = q_dot * pi / 180 # convert q_dot to radians/s

        q_dot = self.saturate_q_dot(q_dot)

        # print 'q_dot', q_dot 

        q = deltaT * q_dot + np.transpose(np.matrix(self.get_angles_arm()))
        return q

    def calc_orient_error(self,quat_cur,quat_des):
        
        quat_des_scalar = float(quat_des[3])
        quat_des_vec = [float(quat_des[0]), float(quat_des[1]), float(quat_des[2])]

        quat_cur_scalar = float(quat_cur[3])
        quat_cur_vec = [float(quat_cur[0]), float(quat_cur[1]), float(quat_cur[2])]

        cross_result = np.cross(quat_des_vec, quat_cur_vec)
        cross_result = np.matrix(cross_result)

        quat_des_vec = np.matrix(quat_des_vec)
        quat_cur_vec = np.matrix(quat_cur_vec)
        
        # orient_error = -q0d * qv + q0 * qvd - cross_result
        orient_error = -quat_des_scalar * quat_cur_vec + quat_cur_scalar * quat_des_vec - cross_result
        orient_error = np.transpose(orient_error)

        return orient_error

    def pos_orient_control(self,deltaT):

        x_current, x_orient = self.get_pose_arm()
        # vel_cur = self.get_vel_arm()

        #Convert EEF position to numpy vector
        x_current = np.matrix(x_current)
        x_orient = np.matrix(x_orient)
        # vel_cur = np.matrix(vel_cur)

        self.pos_error = np.transpose(np.add(self.x_ref,-1.0*x_current))
        # print 'pos_error', self.pos_error

        self.orient_error = self.calc_orient_error(np.transpose(x_orient),np.transpose(self.orient_ref))

        # print 'orientation error \n', self.orient_error        

        J = np.matrix(self.kin.jacobian())
        J_t = np.transpose(J)
        Jp = J[0:3,:]
        # I_6 = np.matrix(np.identity(6), copy=False)
        
        # error vector
        error_vect = np.vstack((self.pos_error, self.orient_error))
        if self.verbose:
            print 'error vector \n', error_vect

        #velocity reference
        vel_ref = np.vstack((np.transpose(self.x_dot_ref),np.matrix([[0],[0],[0]])))
        # print '\n vel_ref \n',vel_ref

        # Jacobian Pseudo-Inverse (Moore-Penrose) + DLS
        # manip = np.linalg.det(J * J_t)
        # beta = pow(np.linalg.norm(error_vect), 2) / (manip * 300)  #numpy.linalg.norm(pos_error)
        # J_pseudoinv = J_t * np.linalg.inv(J * J_t + pow(beta, 2) * I_6)        

        J_reg_p_inv = np.linalg.pinv(J)*pi/180

        q_dot = J_reg_p_inv * (self.K_kin * error_vect + vel_ref)

        q_dot = self.saturate_q_dot(q_dot)
        # print 'q_dot', q_dot

        self.x_dot = Jp*q_dot * pi/180
        self.x_dot = np.transpose(self.x_dot)
        print 'vel', self.x_dot

        q = deltaT * q_dot + np.transpose(np.matrix(self.get_angles_arm()))
        return q

    def saturate_q_dot(self,q_dot):
        
        saturation = self.joint_max_speed

        for k in range(len(q_dot)):
            if (q_dot[k] > saturation):
                q_dot[k] = saturation
            elif (q_dot[k] < -saturation):
                q_dot[k] = -saturation

        return q_dot

    #Execute one control step
    def run(self, x_ref, x_dot_ref, orient_ref=None, verbose = False):

        self.verbose = verbose

        #get current time
        if self.current_time == -1.0:
            self.current_time = rospy.get_time()
            self.old_time = self.current_time
        else:
            self.old_time = self.current_time
            self.current_time = rospy.get_time()

        deltaT = self.current_time - self.old_time

        self.x_ref = x_ref
        self.x_dot_ref = x_dot_ref

        if orient_ref != None:
            # print '\n Controlling EEF position + orientation \n'
            self.orient_ref = orient_ref
            q = self.pos_orient_control(deltaT)

        else:
            # print '\n Controlling EEF position \n'
            q = self.pos_control(deltaT)
        
        if self.verbose:
            print 'Commanded q \n', q_list
        
        q_list = np.squeeze(np.asarray(q)).tolist()


        self.command_msg.names = self.limb.joint_names()
        self.command_msg.command = q_list

        #Publish joint position command
        self.pub_joint_cmd.publish(self.command_msg)