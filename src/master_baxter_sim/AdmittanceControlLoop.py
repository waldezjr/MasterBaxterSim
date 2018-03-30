import rospy
import numpy as np
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
        
        #baxter interface
        self.limb = baxter_interface.Limb(self.limb_name)

        
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #subcriber to get torque measurements
        # rospy.Subscriber('/robot/limb/' + self.limb_name + '/endpoint_state', EndpointState, self.endeffector_callback)
        # rospy.Subscriber('/ft_sensor/' + self.limb_name, WrenchStamped, self.force_sensor_callback)


        #Admittance Controller Parameters
        self.Lambda_d = 2 * np.eye(3, dtype=float)
        self.D_d = 32 * np.eye(3, dtype=float)
        self.K_d0 = 1000 * np.eye(3, dtype=float)

        # Human stiffness max
        self.K_h0 = 2000 * np.eye(3, dtype=float)

        #Max and min values for ICC simulation
        self.icc_MAX = 0.5
        self.icc_min = 0.05

        #Current and old time parameter
        self.current_time = -1.0
        self.old_time = 0.0

        #Current Pose

        self.force_measured = Vector3()

        # initialize kinematic reference variables
        self.x_ref_dot = np.transpose(np.matrix([0.0,0.0]))
        x0 = self.limb.endpoint_pose()['position']
        self.x_ref = np.transpose(np.matrix([x0.x,x0.y,x0.z]))
        print self.x_ref

    # def force_sensor_callback(self, data):

    #     if self.limb == 'left':
    #         # self.tfBuffer.waitForTransform('base','left_wrist', rospy.Time(0), rospy.Duration(1.0))
    #         eef_transformation = self.tfBuffer.lookup_transform('base','left_wrist', rospy.Time(0))
    #     else:
    #         # self.tfBuffer.waitForTransform('base','right_wrist', rospy.Time(0), rospy.Duration(1.0))            
    #         eef_transformation = self.tfBuffer.lookup_transform('base','right_wrist', rospy.Time(0))

    #     force_measured = data.wrench.force

    #     # print '\nforce at sensor frame', force_measured

    #     force_measured = self.qv_mult(force_measured,eef_transformation.transform.rotation)

    #     self.force_measured = Vector3(force_measured[0],force_measured[1],force_measured[2])

    #     # print '\nforce at base frame', self.force_measured
        

    def get_pose_arm(self):
        pos = self.limb.endpoint_pose()['position']
        # pos: x,y,z ---> vec
        orientation = self.limb.endpoint_pose()['orientation']
        # orient: x,y,z,w ---> vec,scalar
        return pos,orientation

    def admittance(self, deltaT, x_r_dot_dot, e_r, e_r_dot, Fh):
        

        # xRef_dot_dot = xRDotDot + inv(Md)*( Fh -D*(xEDot - xRDot ) - Kd*(xE - xR) );
        # %integrate xRef_dot_dot, and xRef_dot
        # xRef_dot = Ts * xRef_dot_dot + xRef_dot;
        # xRef = Ts*xRef_dot+ xRef;

        # get ref acceleration
        self.x_ref_dot_dot = x_r_dot_dot + np.inv(self.Lambda_d)*(Fh - self.D_d*e_r_dot - self.K_d *e_r)
        # integrate to get ref velocity
        self.x_ref_dot = deltaT * self.x_ref_dot_dot + self.x_ref_dot
        # integrate to get ref position
        self.x_ref = deltaT * self.x_ref_dot + self.x_ref

    def calc_alpha(self,e_h):
        # Estimate ICC with sigmoid
        # sigmoid =(iccMax-iccMin) * 1/(1+exp(-(600*norm(xR-xH)-6))) + iccMin;

        # calculate norm(e_h)
        norm_e_h = sqrt((np.transpose(e_h)*e_h)[0,0])
        icc = (self.icc_MAX - self.icc_min)*1/(1+exp(-(600*norm_e_h-6))) + self.icc_min

        self.alpha = (icc - self.icc_min) / (self.icc_MAX - self.icc_min);



    #Execute one control step
    def run(self, x_r, x_r_dot,x_r_dot_dot,x_current_dot,x_h):


        #get current time
        if self.current_time == -1.0:
            self.current_time = rospy.get_time()
            self.old_time = self.current_time
        else:
            self.old_time = self.current_time
            self.current_time = rospy.get_time()

        x_current, x_orient = self.get_pose_arm()
        #Convert EEF position to np vector
        x_current = np.matrix(x_current)
        x_orient = np.matrix(x_orient)
        x_h = np.matrix(x_h)

        self.robot_error = np.transpose(np.add(x_current,-1.0*x_r))
        robot_error_dot = np.transpose(np.add(x_current_dot,-1.0*x_r_dot))

        self.human_error = np.transpose(np.add(x_current,-1.0*x_h))

        deltaT = self.current_time - self.old_time

        # Ideally, we should obtain the external force from a F/T sensor or estimate it from the joint torques
        # However, for simulation purposes we use a virtual human force modelled as a spring

        # self.calc_alpha(self.human_error)
        self.alpha = 0 #robot as leader
        # self.alpha = 1 #robot as follower

        F_h = self.K_h0 * self.alpha * self.human_error

        self.K_d = self.K_d0 * (1-self.alpha) + 10*np.eye(3, dtype=float)

        self.admittance(deltaT,x_r_dot_dot,self.robot_error,self.robot_error_dot,F_h)
