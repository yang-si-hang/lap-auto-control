# coding=utf-8
# created by ysh in 2021/12/1

import os.path
import time
import numpy as np
import pygmo.core as pg
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import scipy.io
from spatialmath.base import *
from math3d.transform import Transform as Trans
import rospy
from sensor_msgs.msg import Joy
# from RcmControl import GetSpeed, MotionControl
np.set_printoptions(precision=5,suppress=True)


"""

"""

path = os.path.dirname(__file__)

IntrinsicMatrix = np.loadtxt(f'{path}/../data/mtx.csv')
IntrinsicMatrix_inv = np.linalg.inv(IntrinsicMatrix)
camera_tool = np.loadtxt(f'{path}/../data/camera_tool.csv')
dist = np.loadtxt(f'{path}/../data/dist.csv')




#参数
T_0_rcm = transl(0.56, 0.30, 0.18) @ troty(np.pi)  @ trotz(np.pi)
T_rcm_0 = np.linalg.inv(T_0_rcm)
q0 = np.pi / 6
# left_base = np.array([[-0.9981, 0.06149, -0.004677, 0.01738],
#                       [0.04068, 0.7135, 0.6995, 0.2161],
#                       [0.04635, 0.698, -0.7146, 1.534],
#                       [0, 0, 0, 1]])
# left_base = left_base @ trotz(-np.pi)
# left_base_inv = np.linalg.inv(left_base)
# right_base = transl(0.0005, -0.22663, 1.53885) @ rpy2tr(2.3446, 0.0426, -0.0476)
right_base = np.identity(4)
# right_base = right_base @ trotz(-np.pi)
right_base_inv = np.linalg.inv(right_base)
T_r_b = T_rcm_0 @ right_base
T_0c_before_deltax = np.identity(4) #求解到最佳姿态的变换矩阵（在状态参量x叠加手柄delta之前）
T_between_delta_0c = np.identity(4) #T_between_delta_0c @ T_0c = T_0c_before_deltax

# 相机的左右、上下视场的一半多
deg_lr = 40
deg_ud = 20
# deg_lr = 50
# deg_ud = 30

#高斯分布参数 [       ,               ,               ,           ,   横坐标      ， 纵坐标       ，          ]
# fun_left = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 473.16764162, 0.00402319]
# fun_right = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 468.94976255, 0.01276680]
# fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 600.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 600.94976255, 0.01276680]
fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 760.18590305, 650.16764162, 0.00402319]
fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1160.58609198, 650.94976255, 0.01276680]  # 原点在右下角
fun_left = fun_left_0.copy()
fun_right = fun_right_0.copy()
threshold = 0.8
# threshold = 0.9

joy_last_time = 0

k_fun_zoom = 200
k_fun_xy = 100

img_width = 1920
img_height = 1080

img_center_x = img_width /2
img_center_y = img_height /2

img_edge_x = 100
img_edge_y = 50

joy_delta_x = [0,0,0]  #
k_angle_xy = 0.1
k_angle_z = 0.01


#手柄切换运镜模式的标识
lap_control_mode = 0  #0:手动运镜，1：自动运镜+手动调节
lock = True
lock_last_button = 0
dx = dy = dz = 0
manual = False
auto_fun = False
auto_angle = True
manual_last_button = 0
auto_fun_last_button = 0
auto_angle_last_button = 0
joy_reset_last_button = False


left_feature_detect = None
right_feature_detect = None
T_r_s = None
T_b_c_now = None


def LeftPosProcess(msg):
    global left_feature_detect
    left_feature_detect = msg.data


def RightPosProcess(msg):
    global right_feature_detect
    right_feature_detect = msg.data


def ShaftPoseProcess(msg):
    global T_r_s, T_b_c_now
    T_b_s = Trans.get_array(Trans(msg.data))
    T_r_s = T_r_b @ T_b_s
    T_b_c_now = T_b_s @ trotx(-q0)


def GetIntersection(set):
    """
    求集合的交集
    :param set: ndarray: N*2
    :return:
    """
    set_min = set[:,0]
    set_max = set[:,1]
    set_min = np.max(set_min)
    set_max = np.min(set_max)

    return [set_min, set_max]



# alpha_bound, beta_bound = feasible_set(np.squeeze(left_pos), np.squeeze(right_pos))
def feasible_set(left_feature, right_feature):
    """
    确保特征出现在相机视野中，相机位姿的可行空间
    :param left_feature: np.array:4
    :param right_feature: np.array:4
    :return:
    """
    T_camera0_0 = T_rcm_0

    left_feature_rcm = T_camera0_0 @ left_feature
    right_feature_rcm = T_camera0_0 @ right_feature

    # alpha为X轴，beta为Y轴
    # X轴需要根据腹腔镜转角增加相应的角度
    left_alpha = -np.arctan2(left_feature_rcm[1],left_feature_rcm[2]) + q0
    left_beta = np.arctan2(left_feature_rcm[0],left_feature_rcm[2])

    right_alpha = -np.arctan2(right_feature_rcm[1],right_feature_rcm[2]) + q0
    right_beta = np.arctan2(right_feature_rcm[0],right_feature_rcm[2])

    alpha_set = np.array([[left_alpha-np.deg2rad(deg_ud), left_alpha+np.deg2rad(deg_ud)], [right_alpha-np.deg2rad(deg_ud), right_alpha+np.deg2rad(deg_ud)]])
    beta_set = np.array([[left_beta-np.deg2rad(deg_lr), left_beta+np.deg2rad(deg_lr)], [right_beta-np.deg2rad(deg_lr), right_beta+np.deg2rad(deg_lr)]])

    alpha_bound = GetIntersection(alpha_set)
    beta_bound = GetIntersection(beta_set)
    # alpha_bound = [alpha_bound+np.pi/6,alpha_bound[1]+np.pi/6]

    return alpha_bound, beta_bound


# fun_now_temp, figure_theta_temp = GetFun(T_r_c, left_feature, right_feature)
def GetFun(T, left, right):
    T_0c = T_0_rcm @ T
    print('T0c:',T_0c)

    R_0c = T_0c[0:3, 0:3]
    figure_theta = -np.arctan2(R_0c[1, 0], R_0c[1, 1])
    # T_0c = T_0c @ trotz(figure_theta)#?????

    T_c0 = np.linalg.inv(T_0c)

    # left = left * np.array([-1,-1,1,1])
    # right = right * np.array([-1,-1,1,1])

    p_cl = np.dot(T_c0, left.T)
    p_cr = np.dot(T_c0, right.T)
    print('left:',left)
    print(f'p cl r:\n{p_cl}, {p_cr}')



    image_left = np.dot(IntrinsicMatrix, (p_cl[0:3] / p_cl[2]))
    image_right = np.dot(IntrinsicMatrix, (p_cr[0:3] / p_cr[2]))
    print(f'image l r:\n{image_left}, {image_right}')

    pixel_left = np.squeeze(np.int32(image_left[0:2]))
    pixel_right = np.squeeze(np.int32(image_right[0:2]))
    print(f'pixel:\n{pixel_left},{pixel_right}')

    J_left = fun_left[6] + fun_left[0] * np.exp(
        -(((pixel_left[0] - fun_left[4]) * np.cos(fun_left[1] * np.pi / 180) + (
                pixel_left[1] - fun_left[5]) * np.sin(fun_left[1] * np.pi / 180)) / fun_left[2]) ** 2 \
        - ((-(pixel_left[0] - fun_left[4]) * np.sin(fun_left[1] * np.pi / 180) + (
                pixel_left[1] - fun_left[5]) * np.cos(fun_left[1] * np.pi / 180)) / fun_left[3]) ** 2)
    J_right = fun_right[6] + fun_right[0] * np.exp(
        -(((pixel_right[0] - fun_right[4]) * np.cos(fun_right[1] * np.pi / 180) + (
                pixel_right[1] - fun_right[5]) * np.sin(fun_right[1] * np.pi / 180)) / fun_right[2]) ** 2 \
        - ((-(pixel_right[0] - fun_right[4]) * np.sin(fun_right[1] * np.pi / 180) + (
                pixel_right[1] - fun_right[5]) * np.cos(fun_right[1] * np.pi / 180)) / fun_right[3]) ** 2)

    J = J_left + J_right

    return -J, figure_theta



# fun_evaluate_temp, figure_theta_evaluate_temp, x_temp = main_pg(left_feature, right_feature)
# 根据左右器械位置和设定的视角范围，求出腹腔镜两角度自由度可行范围，并得出最佳的姿态
def main_pg(left_pos, right_pos):

    global joy_delta_x
    global T_0c_before_deltax, T_between_delta_0c

    # matlab_data = scipy.io.loadmat('20211101_gauss_result.mat')
    # z_image_left = matlab_data['z_image_left']
    # z_image_right = matlab_data['z_image_right']

    # fun_left = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 473.16764162, 0.00402319]
    # fun_right = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 468.94976255, 0.01276680]

    # left_feature = np.array([[-0.1, 0.3, 0., 1]])
    # right_feature = np.array([[0.02, 0.3, 0., 1]])
    # left_feature = np.array([[0.028169, -0.025886, -2.05e-04, 1]])
    # right_feature = np.array([[0.089398, -0.045767, 0.021503, 1]])
    # left_feature = left_pos
    # right_feature = right_pos

    alpha_bound, beta_bound = feasible_set(np.squeeze(left_pos), np.squeeze(right_pos))
    print(alpha_bound)
    print(beta_bound)

    class fun:
        def __init__(self, dim):
            self.dim = dim

        def fitness(self, x):
            z = x[0]
            theta_x = x[1]
            theta_y = x[2]
            theta_z = 0.

            T_0c = T_0_rcm @ trotx(theta_x) @ troty(theta_y) @ trotz(theta_z) @ transl(0, 0, z) @ trotx(-q0)

            R_0c = T_0c[0:3, 0:3]
            figure_theta = -np.arctan2(R_0c[1, 0], R_0c[1, 1])
            # T_0c = T_0c @ trotz(figure_theta) #??????



            T_c0 = np.linalg.inv(T_0c)

            p_cl = np.dot(T_c0, left_pos.T)
            p_cr = np.dot(T_c0, right_pos.T)

            image_left = np.dot(IntrinsicMatrix, (p_cl[0:3] / p_cl[2]))
            image_right = np.dot(IntrinsicMatrix, (p_cr[0:3] / p_cr[2]))

            pixel_left = np.squeeze(np.int32(image_left[0:2]))
            pixel_right = np.squeeze(np.int32(image_right[0:2]))

            J_left = fun_left[6] + fun_left[0] * np.exp(
                -(((pixel_left[0] - fun_left[4]) * np.cos(fun_left[1] * np.pi / 180) + (
                            pixel_left[1] - fun_left[5]) * np.sin(fun_left[1] * np.pi / 180)) / fun_left[2]) ** 2 \
                - ((-(pixel_left[0] - fun_left[4]) * np.sin(fun_left[1] * np.pi / 180) + (
                            pixel_left[1] - fun_left[5]) * np.cos(fun_left[1] * np.pi / 180)) / fun_left[3]) ** 2)
            J_right = fun_right[6] + fun_right[0] * np.exp(
                -(((pixel_right[0] - fun_right[4]) * np.cos(fun_right[1] * np.pi / 180) + (
                            pixel_right[1] - fun_right[5]) * np.sin(fun_right[1] * np.pi / 180)) / fun_right[2]) ** 2 \
                - ((-(pixel_right[0] - fun_right[4]) * np.sin(fun_right[1] * np.pi / 180) + (
                            pixel_right[1] - fun_right[5]) * np.cos(fun_right[1] * np.pi / 180)) / fun_right[3]) ** 2)

            J = J_left + J_right

            return [-J]

        def get_bounds(self):
            # return [(0,0.2),(-np.pi/3,np.pi/3),(-np.pi/3,np.pi/3),(-np.pi,np.pi)]
            # return ([0, -np.pi/3, -np.pi/3, -np.pi/2], [0.4, np.pi/3, np.pi/3, np.pi/2])
       
            return ([0.01, alpha_bound[0], beta_bound[0]], [0.12, alpha_bound[1], beta_bound[1]])

    # fun1 = fun(4)
    # fun1.fitness([0,0,0,0])

    start_time = time.perf_counter()
    prob = pg.problem(fun(3))

    uda = pg.de(gen=500, ftol=5e-3)
    algo = pg.algorithm(uda)
    # algo.set_verbosity(1)
    pop = pg.population(prob, 25)
    pop = algo.evolve(pop)
    # print(pop)
    end_time = time.perf_counter()
    # print(f'using time:{(end_time - start_time):.4f}')
    best_x = pop.champion_x
    T_0c_before_deltax = T_0_rcm @ trotx(best_x[1]) @ troty(best_x[2]) @ transl(0, 0, best_x[0]) @ trotx(-q0)
    print(f'solve x:{best_x} + {joy_delta_x}')
    best_x[0] += joy_delta_x[2]
    best_x[1] += joy_delta_x[1]
    best_x[2] += joy_delta_x[0]
    print(f'total x:{best_x}')

    # print(f'solve fun:{pop.champion_f}')

    T_0c = T_0_rcm @ trotx(best_x[1]) @ troty(best_x[2]) @ transl(0, 0, best_x[0]) @ trotx(-q0)
    R_0c = T_0c[0:3, 0:3]
    T_between_delta_0c = T_0c_before_deltax @ np.linalg.inv(T_0c)
    figure_theta = -np.arctan2(R_0c[1, 0], R_0c[1, 1])
    # print(f'figure rotation:{np.rad2deg(figure_theta):.4f}')

    # fun_0 = fun(3)
    # print(fun_0.fitness(best_x))

    return pop.champion_f[0], np.rad2deg(figure_theta), best_x




def joy_fun_adjust(msg):

    global lock
    global lock_last_button
    global dx,dy,dz
    global manual, auto_fun, auto_angle
    global manual_last_button, auto_fun_last_button, auto_angle_last_button, joy_reset_last_button
    global fun_left, fun_right
    global k_fun_xy, k_fun_zoom
    global fun_left_0, fun_right_0
    global joy_last_time
    global joy_delta_x

    delta_time = time.time() - joy_last_time
    joy_last_time = time.time()
    # print('delta time: ',delta_time)

    if msg.buttons[10]:
        if abs(lock_last_button) < 0.01:
            if lock:
                lock = False
                print('已解锁')
            else:
                lock = True
                print('已锁定')

    lock_last_button = msg.buttons[10]
    if lock:
        dx = dy = dz = 0
        return

    lock_last_button = msg.buttons[10]
    
    threshold = 0.3
    if abs(msg.axes[0]) > threshold:
        dx = -msg.axes[0]
    else: 
        dx = 0

    if abs(msg.axes[1]) > threshold:
        dy = msg.axes[1]
    else:
        dy = 0

    if abs(msg.axes[5]) > threshold:
        dz = msg.axes[5]
    else:
        dz = 0

    if dx or dy or dz :
        print(f'joy msg: ({dx:.2f},{dy:.2f},{dz:.2f})')
        if auto_fun:
            if dz:
                left_x = fun_left[4] - img_center_x
                left_y = fun_left[5] - img_center_y
                left_y_x = left_y / left_x
                left_distance = np.sqrt(pow(left_x,2) + pow(left_y,2))
                right_x = fun_right[4] - img_center_x
                right_y = fun_right[5] - img_center_y
                right_y_x = right_y / right_x
                right_distance = np.sqrt(pow(right_x,2) + pow(right_y,2))
                print(f'left : ({left_x}, {left_y})  {left_distance}')
                print(f'right: ({right_x}, {right_y})  {right_distance}')

                left_x = left_x + left_x / (left_distance + right_distance)  * dz * k_fun_zoom * delta_time
                right_x = right_x + right_x  / (left_distance + right_distance) * dz * k_fun_zoom * delta_time
                left_y = left_y_x * left_x
                right_y = right_y_x * right_x
                fun_left[4] = left_x + img_center_x
                fun_left[5] = left_y + img_center_y
                fun_right[4] = right_x + img_center_x
                fun_right[5] = right_y + img_center_y 
            
            


            if dx:
                fun_left[4] = fun_left[4] + k_fun_xy * dx * delta_time
                fun_right[4] = fun_right[4] + k_fun_xy * dx * delta_time

            if dy:
                fun_left[5] = fun_left[5] + k_fun_xy * dy * delta_time
                fun_right[5] = fun_right[5] + k_fun_xy * dy * delta_time


            if fun_left[4] < img_edge_x:
                fun_left[4] = img_edge_x
            if fun_left[5] < img_edge_y:
                fun_left[5] = img_edge_y

            if fun_right[4] < img_edge_x:
                fun_right[4] = img_edge_x
            if fun_right[5] < img_edge_y:
                fun_right[5] = img_edge_y

            if fun_left[4] > img_width-img_edge_x:
                fun_left[4] = img_width-img_edge_x
            if fun_left[5] > img_height-img_edge_y:
                fun_left[5] = img_height-img_edge_y

            if fun_right[4] > img_width-img_edge_x:
                fun_right[4] = img_width-img_edge_x
            if fun_right[5] > img_height-img_edge_y:
                fun_right[5] = img_height-img_edge_y
            
            # print('fun_L :',fun_left)
            # print('fun_R :',fun_right)

        if auto_angle:
            if dx:
                joy_delta_x[0] += dx * k_angle_xy * delta_time

            if dy:
                joy_delta_x[1] += dy * k_angle_xy * delta_time
            
            if dz:
                joy_delta_x[2] += dz * k_angle_z * delta_time

        




    
    if (not auto_fun_last_button) and msg.buttons[7]:
        manual = False
        auto_angle = False
        auto_fun = True
        print('Auto_fun')
    if (not auto_angle_last_button) and msg.buttons[9]:
        manual = False
        auto_fun = False
        auto_angle = True
        print('Auto_angle')
    if (not manual_last_button) and msg.buttons[6]:
        auto_fun = False        
        auto_angle = False
        manual = True
        print('Manual')
    
    auto_fun_last_button = msg.buttons[7]
    auto_angle_last_button = msg.buttons[9]
    manual_last_button = msg.buttons[6]
    

    if (not joy_reset_last_button) and msg.buttons[11]:
        if auto_fun:
            fun_left = fun_left_0.copy()
            fun_right = fun_right_0.copy()
            print('fun 复位')
            print('fun_L :',fun_left)
            print('fun_L0 :',fun_left_0)
            print('fun_R :',fun_right)
            print('fun_R0 :',fun_right_0)

        if auto_angle:
            joy_delta_x = [0,0,0]

    joy_reset_last_button = msg.buttons[11]

    
    

    if lap_control_mode == 0: #0:手动运镜，1：自动运镜+手动调节
        pass

    # print("-------------------------------------")
    # print(msg.axes[0])
    # print(msg.axes[1])
    # print(msg.axes[2])
    
    

    if lap_control_mode == 0: #0:手动运镜，1：自动运镜+手动调节
        pass

    # print("-------------------------------------")
    # print(msg.axes[0])
    # print(msg.axes[1])
    # print(msg.axes[2])


if __name__ == '__main__':
    rospy.init_node('Optimal', anonymous=True)
    rospy.Subscriber('LeftPos', numpy_msg(Floats), LeftPosProcess)
    rospy.Subscriber('RightPos', numpy_msg(Floats), RightPosProcess)
    rospy.Subscriber('ShaftPose', numpy_msg(Floats), ShaftPoseProcess)
    rospy.Subscriber("joy", Joy ,joy_fun_adjust,queue_size=1)
    desire_pose_pub = rospy.Publisher('DesirePose', numpy_msg(Floats), queue_size=1)
    figure_theta_pub = rospy.Publisher('FigureTheta', Floats, queue_size=1)


    # 世界坐标系：X轴朝左，Y轴朝下，Z轴朝上
    fun_now = []
    fun_evaluate = []
    figure_theta = []
    desire_flag = []
    # 相机的姿态
    # pose_camera = np.zeros([N,6])
    # 图像的姿态，旋转了FigureRotation
    # pose_figure = np.zeros([N,6])
    # 腹腔镜轴在rcm坐标系的位姿
    print('start')
    joy_last_time = time.time()

    while  not rospy.is_shutdown():
        start0 = time.time()
        # print(f'-----------{i}--------------')
        if (T_r_s is None) or (left_feature_detect is None) or (right_feature_detect is None):
            time.sleep(0.01)
            # x_temp = np.array([0.09, 0., 0])
            # desire_pose_pub.publish(x_temp.astype(np.float32))
            continue
        # T_shaft_now = T_0_c @ trotx(q0)
        left_feature = np.hstack((left_feature_detect, np.array([1])))
        right_feature = np.hstack((right_feature_detect, np.array([1])))
        
        T_r_c = T_r_s @ trotx(-q0)
        # fun_now_temp, figure_theta_temp = GetFun(T_r_c, left_feature, right_feature)  #无手柄偏置时的运算
        # fun_temp_old = fun_now_temp
        # print('T_between_delta_0c:\n',T_between_delta_0c)
        T_r_c_before = T_rcm_0 @ T_between_delta_0c @ T_b_c_now
        fun_now_temp, figure_theta_temp = GetFun(T_r_c_before, left_feature, right_feature) #有手柄偏置时的运算
        fun_evaluate_temp, figure_theta_evaluate_temp, x_temp = main_pg(left_feature, right_feature)
        if fun_now_temp > threshold*fun_evaluate_temp:
            T_shaft_desire = trotx(x_temp[1]) @ troty(x_temp[2]) @ transl(0, 0, x_temp[0])
            desire_pose_pub.publish(x_temp.astype(np.float32))
            figure_theta_pub.publish(figure_theta_evaluate_temp)
            desire_flag_temp = 1
        else:
            desire_flag_temp = 0
            T_shaft_desire = 0

        # fun_now.append(fun_now_temp)
        # fun_evaluate.append(fun_evaluate_temp)
        # figure_theta.append(figure_theta_temp)
        # desire_flag.append(desire_flag_temp)
        # x[i] = x_temp
        # [R, t] = tr2rt(T_r_c)
        # rpy = tr2rpy(R)
        # pose_camera[i] = np.hstack((t, rpy))
        # T_r_f = T_r_c @ trotz(figure_theta_temp)
        # [R, t] = tr2rt(T_r_f)
        # rpy = tr2rpy(R)
        # pose_figure[i] = np.hstack((t, rpy))

        # T_shaft_now = T_shaft_next

        # print(f'old fun: {fun_temp_old:.5f}     best fun: {fun_evaluate_temp:.5f}')
        print(f'now fun: {fun_now_temp:.5f}     best fun: {fun_evaluate_temp:.5f}')
        print(f'total:{time.time()-start0}')
        # print(f'best fun: {fun_evaluate_temp:.5f}')
    # np.savetxt(f'{path}/Robotdata/FunNow.csv', np.array(fun_now), fmt='%.6f')
    # np.savetxt(f'{path}/Robotdata/FunEvaluate.csv', np.array(fun_evaluate), fmt='%.6f')
    # np.savetxt(f'{path}/Robotdata/FigureRotation.csv', figure_theta, fmt='%.4f')
    # np.savetxt(f'{path}/Robotdata/DesireFlag.csv', desire_flag, fmt='%i')
    # np.savetxt(f'{path}/Robotdata/X.csv', x, fmt='%.5f')
    # np.savetxt(f'{path}/Robotdata/PoseCamera.csv', pose_camera, fmt='%.5f')
    # np.savetxt(f'{path}/Robotdata/PoseFigure.csv', pose_figure, fmt='%.5f')
