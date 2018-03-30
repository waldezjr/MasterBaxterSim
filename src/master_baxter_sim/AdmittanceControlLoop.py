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

class AdmittanceControlLoop:
    def __init__(self,limb_name):

        #Attributes Initialization
        
        #Name of the limb self is controlling
        self.limb_name = limb_name
        rospy.loginfo("Initializing Baxter's %s arm admittance control", self.limb_name)
        
        #baxter interfac
        self.limb = baxter_interface.Limb(self.limb_name)
        self.kin = baxter_kinematics(self.limb_name)

        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #subcriber to get torque measurements
        # rospy.Subscriber('/robot/limb/' + self.limb_name + '/endpoint_state', EndpointState, self.endeffector_callback)
        rospy.Subscriber('/ft_sensor/' + self.limb_name, WrenchStamped, self.force_sensor_callback)
        # /robot/limb/left/gravity_compensation_torques


        #Admittance Controller Parameters
        self.Lambda_d = 120 * np.eye(3, dtype=float)
        self.D_d = 120 * np.eye(3, dtype=float)
        self.K_d = 120 * np.eye(3, dtype=float)

        #Current and old time parameter
        self.current_time = -1.0
        self.old_time = 0.0

        #Current Pose

        self.end_effector_position = self.limb.endpoint_pose()['position']
        self.end_effector_orient = self.limb.endpoint_pose()['orientation']

        self.force_measured = Vector3()

    def force_sensor_callback(self, data):

        if self.limb == 'left':
            # self.tfBuffer.waitForTransform('base','left_wrist', rospy.Time(0), rospy.Duration(1.0))
            eef_transformation = self.tfBuffer.lookup_transform('base','left_wrist', rospy.Time(0))
        else:
            # self.tfBuffer.waitForTransform('base','right_wrist', rospy.Time(0), rospy.Duration(1.0))            
            eef_transformation = self.tfBuffer.lookup_transform('base','right_wrist', rospy.Time(0))

        force_measured = data.wrench.force

        # print '\nforce at sensor frame', force_measured

        force_measured = self.qv_mult(force_measured,eef_transformation.transform.rotation)

        self.force_measured = Vector3(force_measured[0],force_measured[1],force_measured[2])

        # print '\nforce at base frame', self.force_measured
        

    def get_pose_arm(self):
        pos = self.limb.endpoint_pose()['position']
        # pos: x,y,z ---> vec
        orientation = self.limb.endpoint_pose()['orientation']
        # orient: x,y,z,w ---> vec,scalar
        return pos,orientation

    def admittance(self,deltaT, e_r,e_r_dot,e_r_dot_dot,e_h):
        
        x_current, x_orient = self.get_pose_arm()

        # xRef_dot_dot = xRDotDot + inv(Md)*( Fh -D*(xEDot - xRDot ) - Kd*(xE - xR) );
        # %integrate xRef_dot_dot, and xRef_dot
        # xRef_dot = Ts * xRef_dot_dot + xRef_dot;
        # xRef = Ts*xRef_dot+ xRef;

        #Convert EEF position to np vector
        x_current = np.matrix(x_current)
        x_orient = np.matrix(x_orient)
        # print 'x_current', x_current
        # print 'x_orient', x_orient

        self.robot_error = np.transpose(np.add(self.x_ref,-1.0*x_current))
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
            print '\n Controlling EEF position + orientation \n'
            self.orient_ref = orient_ref
            q = self.pos_orient_control(deltaT)

        else:
            print '\n Controlling EEF position \n'
            q = self.pos_control(deltaT)
        
        if self.verbose:
            print 'Commanded q \n', q_list
        
        q_list = np.squeeze(np.asarray(q)).tolist()


        self.command_msg.names = self.limb.joint_names()
        self.command_msg.command = q_list

        #Publish joint position command
        self.pub_joint_cmd.publish(self.command_msg)