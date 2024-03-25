from geometry_msgs.msg import PoseStamped
import rospy
import time
import numpy as np
import math

pose_now_getted = False
pose_now = []

def state_callback(msg):
    global pose_now
    pose_now = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
    print(f'pose: \n{msg.pose.position.x}\norientation:\n{msg.pose.orientation.w}')
    pose_now_getted = True

def move_circle(pub):
    while not pose_now_getted:
        pass
    radius = 0.05 
    theta = 0
    num_step = 10*1000
    center = np.array([pose_now[0] + radius, pose_now[1], pose_now[2]])
    rate = rospy.Rate(1000)
    for i in range(num_step):
        theta = i/num_step * 2* math.pi
        desire_position = [center[0] - radius*np.cos(theta), center[1]-np.sin(theta), center[2]]
        msg_pub = PoseStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.pose.position.x = desire_position[0]
        msg_pub.pose.position.y = desire_position[1]
        msg_pub.pose.position.z = desire_position[2]
        msg_pub.pose.orientation.w = pose_now[3]
        msg_pub.pose.orientation.x = pose_now[4]
        msg_pub.pose.orientation.y = pose_now[5]
        msg_pub.pose.orientation.z = pose_now[6]
        pub.publish(msg_pub)
        

        pass
    
def speed_scircle():
    while not pose_now_getted:
        pass
    radius = 0.05 
    theta = 0
    num_step = 10*1000
    center = np.array([pose_now[0] + radius, pose_now[1], pose_now[2]])
    rate = rospy.Rate(1000)
    for i in range(num_step):
        theta = i/num_step * 2* math.pi
        desire_position = [center[0] - radius*np.cos(theta), center[1]-np.sin(theta), center[2]]
        msg_pub = PoseStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.pose.position.x = desire_position[0]
        msg_pub.pose.position.y = desire_position[1]
        msg_pub.pose.position.z = desire_position[2]
        msg_pub.pose.orientation.w = pose_now[3]
        msg_pub.pose.orientation.x = pose_now[4]
        msg_pub.pose.orientation.y = pose_now[5]
        msg_pub.pose.orientation.z = pose_now[6]
        pub.publish(msg_pub)


if __name__ == "__main__":

    rospy.init_node('rokae_test', anonymous=True)
    rospy.Subscriber('rokae/state/cartesianPose', PoseStamped, state_callback)
    pub = rospy.Publisher('/rokae/command/CartesianPose1',PoseStamped)
    # time.sleep(0.5)

    move_circle(pub)

    while not rospy.is_shutdown():
        time.sleep(0.1)
