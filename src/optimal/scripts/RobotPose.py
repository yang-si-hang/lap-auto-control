import os.path

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import urx
import time
import logging
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
np.set_printoptions(precision=5, suppress=True)

path = os.path.dirname(__file__)

rcm_point = np.array([0.56, 0.30, 0.18])
q0 = np.pi/6

# left_base = np.array([[-0.9981, 0.06149, -0.004677, 0.01738],
#                       [0.04068, 0.7135, 0.6995, 0.2161],
#                       [0.04635, 0.698, -0.7146, 1.534],
#                       [0, 0, 0, 1]])
# left_base = left_base @ trotz(-np.pi)
# left_base_inv = np.linalg.inv(left_base)
# right_base = transl(0.0005, -0.22663, 1.53885) @ rpy2tr(2.3446, 0.0426, -0.0476)
# right_base = right_base @ trotz(-np.pi)
right_base = np.identity(4)

right_base_inv = np.linalg.inv(right_base)

camera_tcp = np.loadtxt(f'{path}/../data/camera_tool.csv')
# camera_tcp = np.array([[-0.0346, -0.4999, 0.8654, 0.2181],
#                        [-0.9994, 0.0219, -0.0273, 0.0019],
#                        [-0.0053, -0.8658, -0.5003, 0.0324],
#                        [0, 0, 0, 1.0]])
camera_tcp = camera_tcp @ trotx(q0)

rcm_pose = transl(rcm_point) @ troty(np.pi) @ trotz(np.pi)
rcm_pose_inv = np.linalg.inv(rcm_pose)
T_r_b = rcm_pose_inv @ right_base

tcp_pose = Trans.get_pose_vector(Trans(camera_tcp))
tcp_pose = tuple(tcp_pose)


def RCMKinematic(theta_x, theta_y, theta_z, z):
    T = trotx(theta_x) @ troty(theta_y) @ trotz(theta_z) @ transl(0, 0, z)
    return T


def main():
    p1 = right_base_inv @ rcm_pose @ trotx(0.) @ troty(0.) @ trotz(0.) @ transl(0, 0, 0.05)
    pose1 = Trans.get_pose_vector(Trans(p1))

    rospy.init_node('RobotPose', anonymous=True)
    shaft_pose_pub = rospy.Publisher('ShaftPose', numpy_msg(Floats), queue_size=1)
    r = rospy.Rate(30)

    logging.basicConfig(level=logging.WARN)
    rob = urx.Robot("192.168.100.102")
    # rob.set_tcp((0,0,0,0,0,0))
    rob.set_tcp(tcp_pose)
    rob.set_payload(0.5, (0, 0, 0))

    try:
        while not rospy.is_shutdown():
            pose = rob.getl()
            pose_array = np.array(pose, dtype=np.float32)
            # T_b_s = Trans.get_array(Trans(pose_array))
            # T_0_c = left_base @ T_b_s @ trotx(-q0)
            shaft_pose_pub.publish(pose_array)
            r.sleep()
            # T_b_e = Trans.get_array(Trans(pose_array))
            # T_r_e = left_base @ T_b_e
            # print(T_r_e)

    except KeyboardInterrupt:
        rob.close()
    finally:
        rob.close()
    return


if __name__ == '__main__':
    main()
