import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped

from scipy.spatial.transform import Rotation as R

import rtde_receive
import rtde_control


class URros(object):
    def __init__(self):
        self.robot_ip = "192.168.253.10"
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        self.jp_command = JointState()
        self.j_state = JointState()
        self.jv_command = JointState()
        self.cp_command = PoseStamped()
        self.cp_state = PoseStamped()
        self.cv_command = TwistStamped()
        self.cv_state = TwistStamped()
        self.tcpForce_state = WrenchStamped()

        print(f"当前关节角: {self.rtde_r.getActualQ()}")
        print(f"当前TCP位置: {self.rtde_r.getActualTCPPose()}")
        print(f"当前TCP速度: {self.rtde_r.getTargetTCPSpeed()}")

        rospy.Subscriber("ur10e/command/JointPosition", JointState, self.jp_callback)
        rospy.Subscriber("ur10e/command/JointVelocity", JointState, self.jv_callback)
        rospy.Subscriber("ur10e/command/CartesianPose", PoseStamped, self.cp_callback)
        rospy.Subscriber("ur10e/command/CartesianVelocity", TwistStamped, self.cv_callback)

        self.jsPub = rospy.Publisher("ur10e/state/JointState", JointState, queue_size=1)
        self.cpPub = rospy.Publisher("ur10e/state/CartesianPose", PoseStamped, queue_size=1)
        self.cvPub = rospy.Publisher("ur10e/state/CartesianVelocity", TwistStamped, queue_size=1)
        self.tcpforcePub = rospy.Publisher("ur10e/state/TCPForce", WrenchStamped, queue_size=1)

    def self_check(self):
        if self.rtde_c.isConnected():
            rospy.loginfo(" ------------Robot is Connected!!--------------")
        else:
            rospy.logerr("ERROR! Robot is NOT connected ")
            rospy.logerr("robot status code is %d", self.rtde_c.getRobotMode())
        pass

    def jp_callback(self, msg):
        self.jp_command = msg

    def jv_callback(self, msg):
        self.jv_command = msg

    def cp_callback(self, msg):
        self.cp_command = msg

    def cv_callback(self, msg):
        self.cv_command = msg

    def pub_of_state(self):

        stamp=rospy.Time().now()

        force = self.rtde_r.getActualTCPForce()
        jp = self.rtde_r.getActualQ()
        jv = self.rtde_r.getActualQd()

        cp = self.rtde_r.getActualTCPPose()
        quat = R.from_rotvec([cp[3], cp[4], cp[5]], degrees = False).as_quat()

        cv = self.rtde_r.getActualTCPSpeed()

        self.j_state.header.stamp=stamp
        self.j_state.position = jp
        self.j_state.velocity = jv

        self.cp_state.header.stamp = stamp
        self.cp_state.pose.position.x = cp[0]
        self.cp_state.pose.position.y = cp[1]
        self.cp_state.pose.position.z = cp[2]
        self.cp_state.pose.orientation.x = quat[0]
        self.cp_state.pose.orientation.y = quat[1]
        self.cp_state.pose.orientation.z = quat[2]
        self.cp_state.pose.orientation.w = quat[3]

        self.cv_state.header.stamp = stamp
        self.cv_state.twist.linear.x = cv[0]
        self.cv_state.twist.linear.y = cv[1]
        self.cv_state.twist.linear.z = cv[2]
        self.cv_state.twist.angular.x = cv[3]
        self.cv_state.twist.angular.y = cv[4]
        self.cv_state.twist.angular.z = cv[5]

        self.tcpForce_state.header.stamp = stamp
        self.tcpForce_state.wrench.force.x = force[0]
        self.tcpForce_state.wrench.force.y = force[1]
        self.tcpForce_state.wrench.force.z = force[2]
        self.tcpForce_state.wrench.torque.x = force[3]
        self.tcpForce_state.wrench.torque.y = force[4]
        self.tcpForce_state.wrench.torque.z = force[5]

        self.jsPub.publish(self.j_state)
        self.cpPub.publish(self.cp_state)
        self.cvPub.publish(self.cv_state)
        self.tcpforcePub.publish(self.tcpForce_state)

        return 0

    def robot_move_jp(self):
        jp = self.jp_command.position
        self.rtde_c.moveJ(jp, 1.05, 1.4, True)
        pass

    def robot_move_jv(self):
        jv = self.jv_command.velocity
        self.rtde_c.speedJ(jv, 0.5, 0.002)
        pass

    def robot_move_cp(self):
        rotv = R.from_quat(
            [self.cp_command.pose.orientation.x, self.cp_command.pose.orientation.y, self.cp_command.pose.orientation.z,
             self.cp_command.pose.orientation.w]).as_rotvec()
        cp = [self.cp_command.pose.position.x, self.cp_command.pose.position.y, self.cp_command.pose.position.z,
              rotv[0], rotv[1], rotv[2]]
        self.rtde_c.moveL(cp, 0.5, 0.3)
        return 0

    def robot_move_cv(self):
        cv = [self.cv_command.twist.linear.x, self.cv_command.twist.linear.y, self.cv_command.twist.linear.z,
              self.cv_command.twist.angular.x, self.cv_command.twist.angular.y, self.cv_command.twist.angular.z]
        self.rtde_c.speedL(cv, 0.25, 0.002)
        return 0

    def __del__(self):
        self.rtde_c.stopScript()


def main():
    rospy.init_node("test")
    ur_c = URros()
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        # ur_c.robot_move_jp()
        ur_c.pub_of_state()
        # print(ur_c.j_state)
        rate.sleep()
    return 0


if __name__ == "__main__":
    main()
