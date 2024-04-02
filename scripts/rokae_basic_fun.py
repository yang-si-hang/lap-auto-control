import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
import rospy
import math
import time
from spatialmath.base import *
#  关节角发布(位置和速度)
    # js_pub_ = nh.advertise<sensor_msgs::JointState>("rokae/state/JointState", 1);
    # cp_pub_ = nh.advertise<geometry_msgs::PoseStamped>("rokae/state/CartesianPose", 1);
    # cv_pub_ = nh.advertise<geometry_msgs::TwistStamped>("rokae/state/Twist", 1);
    # cf_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("rokae/state/Wrench", 1);

# jp_sub_ = nh.subscribe("rokae/command/JointPosition", 1, &ROKAE_ROS::jp_cb, this);
# jv_sub_ = nh.subscribe("rokae/command/JointVelocity", 1, &ROKAE_ROS::jv_cb, this);
# jf_sub_ = nh.subscribe("rokae/command/JointTorque", 1, &ROKAE_ROS::jf_cb, this);
# cp_sub_ = nh.subscribe("rokae/command/CartesianPose", 1, &ROKAE_ROS::cp_cb, this);
# cv_sub_ = nh.subscribe("rokae/command/Twist", 1, &ROKAE_ROS::cv_cb, this);
# cf_sub_ = nh.subscribe("rokae/command/Wrench", 1, &ROKAE_ROS::cf_cb, this);

class rokae:
    def __init__(self) -> None:
        print('rokae robot initing...')

        self.pose = self.mine_pose()
        self.JointState = JointState()

        self.__Mode = 'cp'
        self.__pub_cp = rospy.Publisher('/rokae/command/CartesianPose',PoseStamped,queue_size=1)
        self.__pub_cv = rospy.Publisher('/rokae/command/Twist',TwistStamped,queue_size=1)
        self.__pub_jp = rospy.Publisher('rokae/command/JointPosition',JointState,queue_size=1)

        rospy.Subscriber('rokae/state/JointState', JointState, self.JointState_callback)
        rospy.Subscriber('rokae/state/CartesianPose', PoseStamped, self.CarteisianPose_callback)
        

        self._frequency = 1000

        

        while self.pose.position.x is None:
            pass

        self.cp_stop()
        print('rokae robot init finished\n==========================================')



    class mine_pose:
        def __init__(self):
            self.position = self.position_class()
            self.orientation = self.orientation_class()
            self._T_save = np.identity(4)
            self._R_save = np.identity(3)

        class position_class:
            def __init__(self):
                self.x = None
                self.y = None
                self.z = None
            
            def array(self):
                return np.array([self.x, self.y, self.z])

            def __str__(self):
                return f"Position:  (x= {self.x:.4f}, \ty= {self.y:.4f}, \tz= {self.z:.4f})"

        class orientation_class:
            def __init__(self):
                self.w = None
                self.x = None
                self.y = None
                self.z = None

            def array(self,sequence='wxyz'):
                '''
                Args:
                    sequence (str): 'wxyz' or 'xyzw'
                return:
                    np.array()
                '''
                if sequence == 'wxyz':
                    return np.array([self.w, self.x, self.y, self.z])
                elif sequence == 'xyzw':
                    return np.array([self.x, self.y, self.z, self.w])
            
            def __str__(self):
                return f"Orientation:  (w= {self.w:.4f}, \tx= {self.x:.4f}, \ty= {self.y:.4f}, \tz= {self.z:.4f})"
        
        def __str__(self):
            return f"Pose:\n{self.position}\n{self.orientation}"
        
        def T_matrix(self):
            self._T_save[:3,:3] = self.R_matrix()
            self._T_save[:3,3] = np.array([self.position.x,self.position.y,self.position.z])
            return self._T_save

        def R_matrix(self):
            self._R_save = np.array(Rotation.from_quat([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]).as_matrix())
            return self._R_save
        
    def JointState_callback(self,msg):
        self.JointState = msg
        # print(f'{self.JointState}')

    def CarteisianPose_callback(self,msg):
        self.pose.position.x = msg.pose.position.x
        self.pose.position.y = msg.pose.position.y
        self.pose.position.z = msg.pose.position.z
        self.pose.orientation.w = msg.pose.orientation.w
        self.pose.orientation.x = msg.pose.orientation.x
        self.pose.orientation.y = msg.pose.orientation.y
        self.pose.orientation.z = msg.pose.orientation.z


    def cp_stop(self, time=1):
        '''
        发送笛卡尔位姿下的停止信号

        Args:
            time:持续发送停止信号的时长

        '''
        msg_pub = PoseStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.pose.position.x = self.pose.position.x
        msg_pub.pose.position.y = self.pose.position.y
        msg_pub.pose.position.z = self.pose.position.z
        msg_pub.pose.orientation.w = self.pose.orientation.w
        msg_pub.pose.orientation.x = self.pose.orientation.x
        msg_pub.pose.orientation.y = self.pose.orientation.y
        msg_pub.pose.orientation.z = self.pose.orientation.z

        rate = rospy.Rate(self._frequency)
        for i in range(int(self._frequency*time)):
            self.__pub_cp.publish(msg_pub)
            rate.sleep()

    def cv_stop(self, time=1):

        rate = rospy.Rate(self._frequency)
        for i in range(int(self._frequency*time)):
            msg_pub = TwistStamped()
            msg_pub.header.stamp = rospy.Time.now()
            msg_pub.twist.linear.x = 0
            msg_pub.twist.linear.y = 0
            msg_pub.twist.linear.z = 0
            msg_pub.twist.angular.x = 0
            msg_pub.twist.angular.y = 0
            msg_pub.twist.angular.z = 0

            self.__pub_cv.publish(msg_pub)
            rate.sleep()

    def jp_stop(self, time=1):
        rate = rospy.Rate(self._frequency)
        for i in range(int(self._frequency*time)):
            msg_pub = JointState()
            msg_pub.header.stamp = rospy.Time.now()
            msg_pub.position = self.JointState.position
            self.__pub_jp.publish(msg_pub)
            rate.sleep()

    

    def cp_cmd(self,T_desire=None, velocity_linear=0.05, velocity_angular=10.0/180.0*math.pi, wait=True, threshold_linear=5e-5, threshold_quaternion=1e-5):
        '''
        笛卡尔位姿控制
        '''
        msg_pub = PoseStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.pose.position.x = self.pose.position.x
        msg_pub.pose.position.y = self.pose.position.y
        msg_pub.pose.position.z = self.pose.position.z
        msg_pub.pose.orientation.w = self.pose.orientation.w
        msg_pub.pose.orientation.x = self.pose.orientation.x
        msg_pub.pose.orientation.y = self.pose.orientation.y
        msg_pub.pose.orientation.z = self.pose.orientation.z

        if self.__Mode != 'cp':
            self.set_mode('cp')
        
        if T_desire is not None:
            T_start = self.pose.T_matrix()
            R_start = T_start[:3,:3]
            R_desire = T_desire[:3,:3]
            R_delta = R_desire @ R_start.T
            rot = Rotation.from_matrix(R_delta)
            axis_angle = rot.as_rotvec()
            theta = np.linalg.norm(axis_angle)
            # axis = axis_angle/theta

            position_desire = np.squeeze(T_desire[:3,3])
            position_start = np.squeeze(T_start[:3,3])
            displacement = position_desire - position_start
            distance = np.linalg.norm(displacement)
            # displace_dir = displacement/distance
            print(f'displacement: {displacement}')
            

            
        steps = int(max(distance/velocity_linear*self._frequency, theta/velocity_angular*self._frequency))
        rate = rospy.Rate(self._frequency)
        print(f'steps: {steps}')
        if steps > 0:
            for i in range(steps+1):
                if rospy.is_shutdown():
                    return
                displacement_step = float(i)/float(steps) * displacement
                position_step = position_start + displacement_step

                axis_angle_step = float(i)/float(steps) * axis_angle
                R_delta_step = Rotation.from_rotvec(axis_angle_step).as_matrix()
                R_step = R_delta_step @ R_start
                rot_step = Rotation.from_matrix(R_step)
                quaternion_step = np.array(rot_step.as_quat()).squeeze()
            

                # print(f'R_desire:\n{R_desire}')
                # print(f'R_start:\n{R_start}')
                # print(f'R_delta:\n{R_delta}')
                # print(f'axis_angle:\n{axis_angle}')



                msg_pub.header.stamp = rospy.Time.now()
                msg_pub.pose.position.x = position_step[0]
                msg_pub.pose.position.y = position_step[1]
                msg_pub.pose.position.z = position_step[2]
                msg_pub.pose.orientation.w = quaternion_step[3]
                msg_pub.pose.orientation.x = quaternion_step[0]
                msg_pub.pose.orientation.y = quaternion_step[1]
                msg_pub.pose.orientation.z = quaternion_step[2]
                
                # print(f'msg_pub:\n{msg_pub}')
                self.__pub_cp.publish(msg_pub)
                rate.sleep()
                
            if wait is True:
                print('wait for cp_cmd...')
                rate = rospy.Rate(self._frequency)
                error_position = np.linalg.norm(self.pose.position.array()-position_step)
                error_quaternion = np.linalg.norm(self.pose.orientation.array(sequence='xyzw')-quaternion_step)

                while error_position > threshold_linear or error_quaternion > threshold_quaternion:
                    if error_position > threshold_linear:
                        print(f'position_step:{position_step}')
                        print(f'position_cur:{self.pose.position.array()}')
                        print(f'error_position: {error_position}')
                    if error_quaternion > threshold_quaternion:
                        # print(f'quaternion_step:{quaternion_step}')
                        # print(f'quaternion_cur:{self.pose.orientation.array(sequence='xyzw')}')
                        print(f'error_quaternion: {error_quaternion}')
                    error_position = np.linalg.norm(self.pose.position.array()-position_step)
                    error_quaternion = np.linalg.norm(self.pose.orientation.array(sequence='xyzw')-quaternion_step)
                    if rospy.is_shutdown():
                        return
                    msg_pub.header.stamp = rospy.Time.now()
                    self.__pub_cp.publish(msg_pub)
                    rate.sleep()
                    pass
                print('reach cp_cmd')
        return


    def cv_cmd(self,velocity):
        '''
        笛卡尔速度控制
        Args:
            velocity: 6元素列表或向量
        '''
        if self.__Mode != 'cv':
            self.set_mode('cv')

        msg_pub = TwistStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.twist.linear.x = velocity[0]
        msg_pub.twist.linear.y = velocity[1]
        msg_pub.twist.linear.z = velocity[2]
        msg_pub.twist.angular.x = velocity[3]
        msg_pub.twist.angular.y = velocity[4]
        msg_pub.twist.angular.z = velocity[5]
        self.__pub_cv.publish(msg_pub)
        

    def jp_cmd(self,joint_position_list, wait=True, velocity = 45.0/180.0*math.pi, threshold = 2e-2/180*math.pi):
        if self.__Mode != 'jp':
            self.set_mode('jp')

        desire_joints = np.array(joint_position_list)
        start_joints = np.array(self.JointState.position)
        delta_joints = desire_joints- start_joints
        delta_max =np.max(np.abs(delta_joints))
        steps = int(delta_max/velocity * self._frequency)
        if steps == 0:
            steps = 1
        msg_pub = JointState()
        print(f'start_joints: {start_joints}\ndesire_joints:{desire_joints}\ndelta_max:{delta_max:.6f} rad ({delta_max/math.pi*180.0}°)\nsteps:{steps}')
        
        time_start = time.time()
        rate = rospy.Rate(self._frequency)
        for i in range(steps+1):
            msg_pub.header.stamp = rospy.Time.now()
            msg_pub.position = list(start_joints + float(i)/float(steps) * delta_joints)
            self.__pub_jp.publish(msg_pub)
            rate.sleep()
        time_send_finish = time.time()
        if wait is True:
            print('wait for jp_cmd...')
            delt_j = np.linalg.norm(desire_joints - np.array(self.JointState.position))


            while delt_j > threshold and not rospy.is_shutdown():
                delt_j = np.linalg.norm(desire_joints - np.array(self.JointState.position))
                msg_pub.header.stamp = rospy.Time.now()
                msg_pub.position = list(joint_position_list)
                self.__pub_jp.publish(msg_pub)

                # print(f'jp_cmd: {joint_position_list}')
                # print(f'jp_now: {self.JointState.position}')
                # print(f'delta_j: {delt_j} > {threshold}')
                rate.sleep()
            print('reached jp_cmd')
        time_reach = time.time()
        print(f'send time: {(time_reach-time_start)}\nwait time:{(time_reach-time_send_finish)}')




    def set_mode(self,mode):
        '''
        Args:
            mode: 
                'cp':笛卡尔位姿
                'cv':笛卡尔速度
                'jp':关节位置
        '''
        print(f'mode changing {self.__Mode} -> {mode} ......')
        if mode == 'cp':
            self.cp_stop()
            self.__Mode = 'cp'
        elif mode == 'cv':
            self.cv_stop()
            self.__Mode = 'cv'
        elif mode == 'jp':
            self.jp_stop()
            self.__Mode = 'jp'
        print(f'mode changed to {mode}')

    def stop(self):
        
        if self.__Mode == 'cp':
            self.cp_stop()
        elif self.__Mode == 'cv':
            self.cv_stop()
        elif self.__Mode == 'jp':
            self.jp_stop()
            
        


