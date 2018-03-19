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

class KinematicControlLoop3:
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
        self.limb.set_joint_position_speed(2) #max velocity for position control

        #As of now, there is a warning coming from baxter_kinematics
        self.kin = baxter_kinematics(self.limb_name)
        
        #Subscriber to baxter's EEF pose
        # rospy.Subscriber('/robot/limb/' + self.limb_name + '/endpoint_state',
            # EndpointState, self.endeffector_callback)

        #Subscriber for baxter's joints
        # rospy.Subscriber('/robot/joint_states',JointState,
            # self.joint_states_callback)

        #Publisher for baxter's JointCommand
        self.pub_joint_cmd = rospy.Publisher('/robot/limb/' + self.limb_name 
            +'/joint_command', JointCommand, queue_size=1)

        self.command_msg = JointCommand()
        self.command_msg.mode = JointCommand.POSITION_MODE


        #Kinematic Controller Parameters
        self.K_kin = 50 * np.eye(3, dtype=float)

        print self.K_kin

        #Current and old time parameter
        self.current_time = -1.0
        self.old_time = 0.0

        #Current Pose

        self.end_effector_position = self.limb.endpoint_pose()['position']
        self.end_effector_orient = self.rotate_list(self.limb.endpoint_pose()['orientation'],1)

        #Pose Trajectory Reference
        #Position
        self.x_ref = np.matrix([0.8,0,0.15])
        #Velocity
        self.x_dot_ref = np.matrix([0,0,0]) 

        self.kin.print_kdl_chain()
        # print self.kin.inverse_kinematics([0.5,-0.2,0.3])

        print self.get_angles_right_arm()

        # print self.kin.forward_position_kinematics([4.14,-42.12,-31.31,-24.30,6.25,-54.51,46.99])
     
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

        # print 'x_current', x_current

        self.pos_error = np.transpose(np.add(self.x_ref,-1.0*x_current))


        # print 'pos_error', self.pos_error        

        J = np.matrix(self.kin.jacobian())

        # print 'J', J

        Jp = J[0:3,:]
        print 'Jp', Jp        

        JpInv = np.linalg.pinv(Jp)

        # print 'JpInv', JpInv        
        q_dot = JpInv * (np.transpose(self.x_dot_ref) + self.K_kin*(self.pos_error))
        q_dot = q_dot * pi / 180

        print 'q_dot', q_dot 

        deltaT = self.current_time - self.old_time

        # q = deltaT * q_dot + np.transpose(np.matrix(self.actual_angles))
        q = deltaT * q_dot + np.transpose(np.matrix(self.get_angles_right_arm()))

        # print 'q', q

        q_list = np.squeeze(np.asarray(q)).tolist()

        print 'q_list', q_list

        self.command_msg.names = self.limb.joint_names()
        self.command_msg.command = q_list

        #Publish joint position command
        self.pub_joint_cmd.publish(self.command_msg)
