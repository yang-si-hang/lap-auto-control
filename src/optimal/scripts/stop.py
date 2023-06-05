import rospy

from std_msgs.msg import Int8

if __name__ == '__main__':
    rospy.init_node('stop', anonymous=True)
    stop_pub = rospy.Publisher('StopSignal', Int8, queue_size=1)
    msg = Int8()
    msg.data = msg
    while not rospy.is_shutdown():
        stop_pub.publish(msg) 
