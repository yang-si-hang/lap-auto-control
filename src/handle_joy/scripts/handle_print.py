import rospy
from sensor_msgs.msg import Joy




def printjoy(msg):
    print("-------------------------------------")
    print(msg.axes[0])
    print(msg.axes[1])
    print(msg.axes[2])
    print(msg.axes[5])
    print(msg.buttons[4])
    print(msg.buttons[2])




if __name__ == "__main__":

    rospy.init_node("handle_print")
    sub = rospy.Subscriber("joy", Joy ,printjoy,queue_size=1)
    

    rospy.spin()

