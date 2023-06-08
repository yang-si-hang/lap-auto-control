import os.path

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int8
import urx
import time
import logging
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
np.set_printoptions(precision=6, suppress=True)

path = os.path.dirname(__file__)

rcm_point = np.array([0.56, 0.30, 0.18])
# rcm_point = np.array([0.55964715, 0.17523912, 0.21301739])
max_speed = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.01])

# left_base = np.array([[-0.9981, 0.06149, -0.004677, 0.01738],
#                       [0.04068, 0.7135, 0.6995, 0.2161],
#                       [0.04635, 0.698, -0.7146, 1.534],
#                       [0, 0, 0, 1]])
# left_base = left_base @ trotz(-np.pi)
# left_base_inv = np.linalg.inv(left_base)

# right_base = transl(0.0005, -0.22663, 1.53885) @ rpy2tr(2.3446, 0.0426, -0.0476)
# right_base = transl(0, 0, 0) @ rpy2tr(0, 0, 0)
right_base = np.identity(4)
# right_base = right_base @ trotz(-np.pi)
right_base_inv = np.linalg.inv(right_base)

camera_tcp = np.loadtxt(f'{path}/../data/camera_tool.csv')
# camera_tcp = np.array([[-0.0346, -0.4999, 0.8654, 0.2181],
#                       [-0.9994, 0.0219, -0.0273, 0.0019],
#                       [-0.0053, -0.8658, -0.5003, 0.0324],
#                       [0, 0, 0, 1.0]])
camera_tcp = camera_tcp @ trotx(np.pi/6)

rcm_pose = transl(rcm_point) @ troty(np.pi) @ trotz(np.pi)
rcm_pose_inv = np.linalg.inv(rcm_pose)
T_r_b = rcm_pose_inv @ right_base

tcp_pose = Trans.get_pose_vector(Trans(camera_tcp))
tcp_pose = tuple(tcp_pose)

rcm_joint = None
T_b_s = None
shaft_pose = None
stop_signal = None


def DesirePoseProcess(mag):
    global rcm_joint
    rcm_joint = mag.data


def TrackLeftXProcess(mag):
    global rcm_joint
    rcm_joint = mag.data


def ShaftPoseProcess(msg):
    global T_b_s
    global shaft_pose
    shaft_pose = msg.data
    T_b_s = Trans.get_array(Trans(shaft_pose))
    # print(f'{left_base @ T_b_s}')

def StopSignalProcess(msg):
    global stop_signal
    stop_signal = msg.data


def RCMKinematic(theta_x, theta_y, theta_z, z):
    T = trotx(theta_x) @ troty(theta_y) @ trotz(theta_z) @ transl(0,0,z)
    return T


def MoveToDesire(robot, desire_pose, K):
    """
    使腹腔镜在RCM约束下移动到期望位姿
    期望速度包含两部分：
        1、rcm误差产生,只发生在shaft坐标系xy方向
        2、根据期望shaft位置和当前位置的差,xy方向位置差由绕rcm的角速度产生,然后再单独计算所需的绕z轴转动
    :param robot:
    :param desire_pose: RCM坐标系中期望点的位置
    :param K: 控制率中的比例系数
    :return:
    """
    # pose_b_e = robot.getl()
    # T_b_e = Trans.get_array(Trans(pose_b_e))
    T_r_s = T_r_b @ T_b_s
    # print(f'腹腔镜尖端在rcm坐标系的位姿：{T_r_e}')
    T_r_shaft = T_r_s #@ trotx(np.pi/6)
    p_shaft = T_r_shaft[0:3,3]
    v_shaft = T_r_shaft[0:3,2]
    rcm_error = p_shaft - np.dot(p_shaft, v_shaft) * v_shaft
    rcm_error_s = np.linalg.inv(T_r_s[0:3,0:3]) @ rcm_error
    # print(rcm_error)
    rcm_error_v = -2. * rcm_error_s
    L = np.linalg.norm(T_r_shaft[0:3, 3])
    T_desire_s = np.linalg.inv(T_r_s) @ desire_pose
    # print(f'期望姿态在尖端坐标系下的位姿{T_desire_e}')
    # 位置向量
    dp = T_desire_s[0:3, 3]
    # if np.mod(i,10) == 0:
    # 设置位置误差的阈值
    if np.sum(np.abs(dp)) < 0.002:
        return None, None, None
    # print(f'pos error: {dp}')
    # print(f'rcm error:{rcm_error[0:2]}')
    #根据期望shaft位置和当前位置差得到速度，其中xy分量由绕rcm转动的自由度产生
    e_v = K * dp
    e_wx = -(e_v[1]) / L
    e_wy = (e_v[0]) / L

    # 计算两个姿态Z轴旋转的差值
    rot_t = np.cross(np.array([0, 0, 1]), T_desire_s[0:3, 2])
    theta = np.arccos(T_desire_s[2, 2])
    RR = angvec2tr(theta, -rot_t)
    RR = RR @ T_desire_s
    theta_z = np.arctan(RR[1,0]/RR[0, 0])

    e_wz = K * theta_z

    speed = [e_v[0], e_v[1], e_v[2], e_wx, e_wy, e_wz]
    speed_array = np.array(speed)
    div = max_speed/np.abs(speed_array)
    if np.min(div) < 1:
        max_sort = np.argmin(div)
        k = np.abs(max_speed[max_sort] / speed_array[max_sort])
        speed_array = k * speed_array
        # speed = speed_array.tolist()
    speed_send = speed_array + np.array([rcm_error_v[0], rcm_error_v[1], 0., 0., 0., 0.])
    speed = speed_send.tolist()
    # if np.mod(i,10) == 0:
    # print(f'speed: {np.array(speed)}')
    # ???
    robot.my_speedl_tool(speed, 0.3, 0.1)

    return dp, speed, rcm_error_s


def main():
    global stop_signal

    # 初始姿态
    p1 = right_base_inv @ rcm_pose @ trotx(np.pi/4) @ troty(-np.pi/10) @ trotz(0.) @ transl(0, 0, 0.02)

    pose1 = Trans.get_pose_vector(Trans(p1))
    # pose1 = Trans.get_pose_vector(Trans(np.array([  [-0.9459478 , -0.31663491 , 0.07017897  ,0.61898767],
    #                                                 [-0.28070318 , 0.69094978 ,-0.66617875 , 0.19490016],
    #                                                 [ 0.1624453  ,-0.64986978 ,-0.74248285  ,0.32862668],
    #                                                 [ 0.  ,        0.    ,      0.    ,      1.        ]])))

    logging.basicConfig(level=logging.WARN)
    rob = urx.Robot("192.168.100.102")
    # rob.set_tcp((0,0,0,0,0,0))
    rob.set_tcp(tcp_pose)
    rob.set_payload(0.5, (0,0,0))

    v = 0.05
    a = 0.3
    print("======= pose1 ======")
    print(pose1)
    rob.movel(pose1, acc=a, vel=v)
    i = 0

    # exit(0)

    time_record = []
    rcm_joint_record = []
    shaft_pose_record = []
    rcm_error_write = []


    try:
        # f1 = open(f'{path}/Robotdata/RcmJoint.csv', 'w')
        # f2 = open(f'{path}/Robotdata/ShaftPose.csv', 'w')
        # f3 = open(f'{path}/Robotdata/RobotTime.txt', 'w')
        while  not rospy.is_shutdown():
            if rcm_joint is None: #等待 DesirePose 传来的数据
                time.sleep(0.05)
                continue
            else:
                t = rospy.Time.now()
                time_record.append(float(f'{t}'))
                rcm_joint_record.append(rcm_joint)
                shaft_pose_record.append(shaft_pose)

                i = i+1
                T_desire_r = RCMKinematic(rcm_joint[1], rcm_joint[2], 0., rcm_joint[0])
                print("T_desire_r\n",T_desire_r)
                # print(rob)
                pos_error, speed_show, rcm_error = MoveToDesire(rob, T_desire_r, 0.4)
                # pos_error, speed_show, rcm_error = MoveToDesire(rob, T_desire_r, 0.3)
                # if rcm_error is None:
                #     np.savetxt(f'{path}/Robotdata/RcmJoint.csv', np.array(rcm_joint_record), fmt='%.3f')
                #     np.savetxt(f'{path}/Robotdata/ShaftPose.csv', np.array(shaft_pose_record), fmt='%.3f')
                #     np.savetxt(f'{path}/Robotdata/RobotTime.csv', np.array(time_record))
                #     np.savetxt(f'{path}/Robotdata/RcmError00.csv', rcm_error_write)
                #     exit(0)
                # rcm_error_write.append(rcm_error)
                if i == 10:
                    if pos_error is not None:
                        # print(f'pos error: {pos_error}')
                        # print(f'speed: {np.array(speed_show)}')
                        pass
                    i = 0
                # f1.write(f'{rcm_joint_record}\n')
                # f2.write(f'{np.array(shaft_pose_record)}\n')
                # f3.write(f'{time_record}\n')
                # time.sleep(0.05)
                # rob.my_speedl_tool([0, 0, 0., 0.1, 0., 0.], 0.3, 0.1)
            if stop_signal == 1:
                np.savetxt(f'{path}/../data/Robotdata/RcmJoint.csv', np.array(rcm_joint_record), fmt='%.3f')
                np.savetxt(f'{path}/../data/Robotdata/ShaftPose.csv', np.array(shaft_pose_record), fmt='%.3f')
                np.savetxt(f'{path}/../data/Robotdata/RobotTime.csv', np.array(time_record))
                exit(0)

    except KeyboardInterrupt:
        rob.close()
    except SystemExit:
        rob.close()
    finally:
        rob.close()

    return

if __name__ == '__main__':
    rospy.init_node('CameraMove', anonymous=True)
    rospy.Subscriber('DesirePose', numpy_msg(Floats), DesirePoseProcess)
    rospy.Subscriber('StopSignal', Int8, StopSignalProcess)
    # rospy.Subscriber('TrackLeftX', numpy_msg(Floats), TrackLeftXProcess)
    rospy.Subscriber('ShaftPose', numpy_msg(Floats), ShaftPoseProcess)
    main()
