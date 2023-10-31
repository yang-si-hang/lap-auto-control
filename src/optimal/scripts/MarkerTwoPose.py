# coding=utf-8
# created by ysh in 2022/1/17

import copy
import logging
import os.path
import time
import sys

import numpy as np
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from skimage.draw import line
from scipy.optimize import fsolve, brentq, root
from spatialmath.base import *
from math3d.transform import Transform as Trans
# from my_pkg.dual_ur5_kin import LeftUr5, RightUr5

from lap_set_pk import lap_set

sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

np.set_printoptions(precision=3,suppress=True)

path = os.path.dirname(__file__)

IntrinsicMatrix = np.loadtxt(f'{path}/../data/mtx.csv')
IntrinsicMatrix_inv = np.linalg.inv(IntrinsicMatrix)
camera_tool = np.loadtxt(f'{path}/../data/camera_tool.csv')
dist = np.loadtxt(f'{path}/../data/dist.csv')

# coordinate_set_path = f'{os.path.dirname(__file__)}/../data/coordinate_set/'
# left_rcm_pos_file = coordinate_set_path + 'left_rcm_pos.csv'
# right_rcm_pos_file = coordinate_set_path + 'right_rcm_pos.csv'
# left_tip_0_file = coordinate_set_path + 'left_tip_0.csv'
# right_tip_0_file = coordinate_set_path + 'right_tip_0.csv'

marker_area_min = 50
marker_area_max = 9000
d_tip_marker = 0.005
q0 = np.pi/6

# left_tip_0 = np.array([0.55, 0.08, 0.93])
# right_tip_0 = np.array([0.45, 0.08, 0.93])
left_tip_0 = lap_set.left_tip_0
right_tip_0 = lap_set.right_tip_0

beta = 0.9      # 移动加权平均系数

left_rcm_pos = lap_set.left_rcm_pos
right_rcm_pos = lap_set.right_rcm_pos
# left_rcm_pos = np.array([0.45108569, 0.26102381, 0.16898061, 1.])
# right_rcm_pos = np.array([0.67721158, 0.25670649, 0.17017894, 1.])


# left_rcm_pos = np.array([0.59508, 0.22294, 0.99971, 1.])
# right_rcm_pos = np.array([0.43117, 0.22188, 1.00401, 1.])

# left_base = np.array([[-0.9981, 0.06149, -0.004677, 0.01738],
#                       [0.04068, 0.7135, 0.6995, 0.2161],
#                       [0.04635, 0.698, -0.7146, 1.534],
#                       [0, 0, 0, 1]])
# left_base = left_base @ trotz(-np.pi)
# left_base_inv = np.linalg.inv(left_base)
right_base = np.identity(4)
# right_base = right_base @ trotz(-np.pi)
right_base_inv = np.linalg.inv(right_base)
# camera_robot = LeftUr5()
# joint = np.array([[31.68, -106.71, -106.11, -44.32, 104.90, -146.69]])
# joint = np.deg2rad(joint)
# T_0_c = camera_robot.my_fkine(joint)
# T_0_c = T_0_c.A @ camera_tool

# T_0_c = np.identity(4)
T_0_c = None


def ShaftPoseProcess(msg):
    global T_0_c
    T_b_s = Trans.get_array(Trans(msg.data))
    T_0_c = right_base @ T_b_s @ trotx(-q0)
    # print(T_0_c)


def SolveH(rcm_point, point_pixel, p):
    h_orient = IntrinsicMatrix_inv @ np.append(point_pixel, np.array([1]))
    h_orient = h_orient/np.linalg.norm(h_orient)
    rcm_length = np.linalg.norm(rcm_point)
    cos_theta = np.dot(rcm_point, h_orient)/rcm_length
    temp = np.square(rcm_length) * (np.square(cos_theta) - 1) + np.square(p)
    if temp < 0:
        return None
    else:
        point_length = rcm_length*cos_theta + np.sqrt(temp)

    return point_length * h_orient


def SortPoints(points):
    """
    将点按从小到大排序
    :param points:
    :return:
    """
    feature = np.asarray(points)
    feature_x = feature[:,0]
    feature_y = feature[:,1]
    if abs(feature_x[0]-feature_x[-1]) > abs(feature_y[0]-feature_y[-1]):
        sort = np.argsort(feature_x)
    else:
        sort = np.argsort(feature_y)
    points_sort = feature[sort,:]
    # points_sort = np.array(points_sort)

    return np.asarray(points_sort)


def GetTipPos(point_3d, pos_rcm, T_0c):

    point_3d = np.expand_dims(np.append(point_3d, np.array([1])), axis=1)
    point_3d_0 = T_0c @ point_3d
    point_3d_0 = np.squeeze(point_3d_0[0:3])

    pos_rcm = np.squeeze(pos_rcm)
    orient = (point_3d_0 - pos_rcm) / np.linalg.norm(point_3d_0 - pos_rcm)
    # 0.0275为tip到第一个标记点的距离
    tip_pos = point_3d_0 + d_tip_marker * orient
    return tip_pos


def FindPerpendicularFoot(line_param, point):
    """
    找到点相对某条线的垂足
    :param line_param:
    :param point:
    :return:
    """
    orient = line_param[0:2]
    origin_point = line_param[2:4]

    foot = origin_point + orient*np.dot(orient, point-origin_point)

    return foot


def cross_ratio(feature, point_rcm):
    """
    根据特征像素与RCM点的位置求解特征在相机坐标系的三维坐标
    :param feature:
    :param point_rcm:
    :return:
    """
    if feature is None:
        return None, None
    else:
        feature = np.asarray(feature)
        # if feature.shape[0] == 4:
        feature_x = feature[:,0]
        feature_y = feature[:,1]
        if abs(feature_x[0]-feature_x[-1]) > abs(feature_y[0]-feature_y[-1]):
            sort = np.argsort(feature_x)
        else:
            sort = np.argsort(feature_y)
        feature_sort = feature[sort,:]
        feature_sort = np.flipud(feature_sort)
        # print(feature)

        if point_rcm[2] < 0:
            feature_sort_inverse = feature_sort[::-1]
        else:
            feature_sort_inverse = feature_sort[:]
        p_rcm = IntrinsicMatrix @ (point_rcm/point_rcm[2])
        pixel_rcm = np.squeeze(p_rcm[0:2])
        # print(pixel_rcm)
        if np.dot(feature_sort[0]-pixel_rcm, feature_sort[0]-feature_sort[-1]) > 0:
            point1, point2, point3, point4 = feature_sort_inverse[0:4]
        else:
            point4, point3, point2, point1 = feature_sort_inverse[0:4]

        # points_total = np.vstack((feature_sort, pixel_rcm))
        line_param = cv2.fitLine(feature_sort, cv2.DIST_L2, 0, 1e-2, 1e-2)
        line_param = np.squeeze(line_param)

        point1_ = FindPerpendicularFoot(line_param, point1)
        point3_ = FindPerpendicularFoot(line_param, point3)
        point4_ = FindPerpendicularFoot(line_param, point4)
        pixel_rcm_ = FindPerpendicularFoot(line_param, pixel_rcm)

        p_1rcm = np.linalg.norm(point1_-pixel_rcm_)
        p_3rcm = np.linalg.norm(point3_-pixel_rcm_)
        p_14 = np.linalg.norm(point1_-point4_)
        p_34 = np.linalg.norm(point3_-point4_)

        k = (p_1rcm*p_34)/(p_14*p_3rcm)
        # print(f'cross ratio: {k}')

        p = (0.02*0.03/0.01)*k/((0.03/0.01)*k-1)
        print(f'distance: {p}')

        # ???
        if p > 0.5 or p < 0.05:
        # if False:
            return None, None
        else:

         
            point1_3d = SolveH(point_rcm, point1, p)
      

    return point1_3d, point1


def LineDetect(img, dirname):
    """
    检测特征，并返回直线参数
    :param img:
    :param dirname:
    :return:
    """
    start0 = time.time()
    # img_gauss = cv2.GaussianBlur(img, (3,3), 0)
    img_median = cv2.medianBlur(img, 5)
    start1 = time.time()
    # print(start1-start0)
    img_hsv = cv2.cvtColor(img_median, cv2.COLOR_BGR2HSV)
    start2 = time.time()
    # print(start2-start1)
    # 绿色的阈值
    lb = np.array([35, 43, 46])
    ub = np.array([77, 255, 255])

    img_green = cv2.inRange(img_hsv, lb, ub)
    # cv2.imwrite(f'{dirname}_green.png', img_green)
    # cv2.imshow('green', img_green)
    
    start3 = time.time()
    # print(start3-start2)

    # 画轮廓
    img_edge, hierarchy = cv2.findContours(img_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    start4 = time.time()
    # print(start4-start3)
    # empty = np.zeros([1080,1920], np.uint8)
    # edge = cv2.drawContours(empty, img_edge, -1, 255)
    # cv2.imwrite(f'{dirname}_edge.png', edge)

    # 根据面积筛选，返回边界像素
    feature_candidate_all = []
    edge_area = []
    # print(len(img_edge))
    for i in range(len(img_edge)):
        edge_area_temp = cv2.contourArea(img_edge[i][:,0,:])
        if (edge_area_temp > marker_area_min) and (edge_area_temp < marker_area_max):
            feature_candidate_all.append((img_edge[i][:,0,:],edge_area_temp))
            edge_area.append(edge_area_temp)
            # print(edge_area_temp)


    feature_candidate = sorted(feature_candidate_all,key = lambda feature_candidate_all: feature_candidate_all[1], reverse=True)
    start5 = time.time()
    print("len of feature_candidate:",len(feature_candidate))
    # print(start5-start4)
    if len(feature_candidate) < 4:
        return None, None
    else:
        cX = []
        cY = []
        feature_show = []
        if len(feature_candidate) < 8:
            feature_candidate = feature_candidate[0:4]
            for feature, _ in feature_candidate:
                M = cv2.moments(feature)
                cX.append(M["m10"] / M["m00"])
                cY.append(M["m01"] / M["m00"])
                feature_show.append(feature)
                centre_points = np.transpose(np.array([cX, cY]))
                centre_points_x = centre_points[:, 0]
                sort_x = np.argsort(centre_points_x)
                centre_points_sort = centre_points[sort_x, :]
            left_centre_points = np.asarray(centre_points_sort[0:4, :])
            return None, left_centre_points
        else:
            feature_candidate = feature_candidate[0:8]
            # edgearea = feature_candidate[0]
            # print(edgearea[1])
            for feature, _ in feature_candidate:
                M = cv2.moments(feature)
                cX.append(M["m10"] / M["m00"])
                cY.append(M["m01"] / M["m00"])
                feature_show.append(feature)
                centre_points = np.transpose(np.array([cX, cY]))
                centre_points_x = centre_points[:, 0]
                sort_x = np.argsort(centre_points_x)
                centre_points_sort = centre_points[sort_x, :]
            left_centre_points = np.asarray(centre_points_sort[0:4, :])
            right_centre_points = np.asarray(centre_points_sort[4:8,:])
            return left_centre_points, right_centre_points

        # edge_filter = np.zeros([1080,1920], np.uint8)
        # edge_filter = cv2.drawContours(edge_filter, feature_show, -1, 255, -1)
        # cv2.imwrite(f'{dirname}/filter.png', edge_filter)

        # print(centre_points)
        # centre_points_int = centre_points.astype(np.int32)
        # img_center = img
        # for i in range(4):
        #     img_center = cv2.circle(img_center, tuple(centre_points_int[i,:]), 2, (0,0,255), -1)
        # cv2.imwrite(f'{dirname}/center.png', img_center)
        # line_param = cv2.fitLine(centre_points, cv2.DIST_L2, 0, 1e-2, 1e-2)
        # line_param = np.squeeze(line_param)
        # print(line_param)

        # for i in range(centre_points.shape[0]):
        #     centre_points[i] = FindPerpendicularFoot(line_param, centre_points[i])

    #, edge_filter


if __name__ == '__main__':
    rospy.init_node('FigureProcess', anonymous=True)
    left_pub = rospy.Publisher('LeftPos', numpy_msg(Floats), queue_size=1)
    right_pub = rospy.Publisher('RightPos', numpy_msg(Floats), queue_size=1)
    rospy.Subscriber('ShaftPose', numpy_msg(Floats), ShaftPoseProcess)

    cv2.namedWindow('figure', 0)
    cv2.resizeWindow('figure', 900, 540)
    capture = cv2.VideoCapture(lap_set.camera_usb_id)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    capture.set(cv2.CAP_PROP_FPS, 30)
    # print(f'曝光：{capture.get(15)}')
    # print(f'亮度：{capture.get(10)}')
    # print(f'对比度：{capture.get(11)}')
    # print(f'增益：{capture.get(14)}')
    capture.set(10, 255)  # 亮度
    capture.set(14, 4)   # 增益        
    # capture.set(14, 8)   # 增益
    capture.set(15, 30)   # 曝光度

    # print(f'亮度：{capture.get(10)}')
    # capture.set(11, 37)
    capture.set(4, 720)  # 图片宽度
    capture.set(3, 1280)  # 图片宽度
    # capture.set(4, 1080)  # 图片宽度
    # capture.set(3, 1920)  # 图片宽度
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    # fps = 20
    # framesize = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    # outCamera = cv2.VideoWriter('outputCamera.avi', fourcc, fps, framesize)

    # h = 1080
    # w = 1920
    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(IntrinsicMatrix, dist, (w, h), 0, (w, h))  # 自由比例参数
    # mapx,mapy = cv2.initUndistortRectifyMap(IntrinsicMatrix, dist, None, IntrinsicMatrix, (w, h), cv2.CV_16SC2)

    # left_f = open(f'{path}/LeftPos.txt', 'w')
    # right_f = open(f'{path}/RightPos.txt', 'w')

    try:
        while  not rospy.is_shutdown():
            start0 = time.time()
            if T_0_c is None:
                time.sleep(0.05)
                continue
            left_rcm_pos_ = np.linalg.inv(T_0_c) @ left_rcm_pos
            left_rcm_pos_ = np.squeeze(left_rcm_pos_)
            right_rcm_pos_ = np.linalg.inv(T_0_c) @ right_rcm_pos
            right_rcm_pos_ = np.squeeze(right_rcm_pos_)

            ret, frame = capture.read()
            start1 = time.time()
            # print(start1 - start0)
            # img = cv2.imread(f'{path}/test/fig.png')
            # img_dst = cv2.undistort(frame, IntrinsicMatrix, dist, None, newcameramtx)
            # img_dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            img_dst = frame
            start2 = time.time()
            # print(start2-start1)
            # img_dst = frame
            # cv2.imwrite(f'{path}/test/fig.png', img_dst)

            left_gravity_center, right_gravity_center = LineDetect(img_dst, path + '/test/')
            start3 = time.time()
            # print(f'Linedetect total: {start3 - start2}')
            # print(gravity_center)
            if right_gravity_center is None:
                cv2.imshow('figure', img_dst)
                # outCamera.write(img_dst)
                start4 = time.time()
                # time.sleep(0.01)
                cv2.waitKey(1)
                # print(time.time()-start4)
                # print('no marker!')
                continue
            else:
                print("left:")
                left_point_3d_gravity, left_tip_marker_show = cross_ratio(left_gravity_center, left_rcm_pos_[0:3])
                print("right:")
                right_point_3d_gravity, right_tip_marker_show = cross_ratio(right_gravity_center, right_rcm_pos_[0:3])
                if (left_point_3d_gravity is None) and (right_point_3d_gravity is None):
                    cv2.imshow('figure', img_dst)
                    # outCamera.write(img_dst)
                    cv2.waitKey(1)
                    print('wrong height!')
                    continue
                else:
                    if left_point_3d_gravity is not None:
                        left_tip_marker_show = left_tip_marker_show.astype(np.int32)

                        img_dst = cv2.circle(img_dst, tuple(left_tip_marker_show), 4, (0, 0, 255), -1)
                        left_tip_pos_gravity = GetTipPos(left_point_3d_gravity, left_rcm_pos[0:3], T_0_c)
                        left_tip_1 = beta*left_tip_0 + (1-beta)*left_tip_pos_gravity
                        left_tip_0 = left_tip_1
                        left_tip = left_tip_1.astype(np.float32)
                        left_pub.publish(left_tip)
                        # cv2.putText(img_dst, f'[{1000*left_tip_pos_gravity[0]:.1f}, {1000*left_tip_pos_gravity[1]:.1f}, {1000*left_tip_pos_gravity[2]:.1f}]',
                        #             tuple(left_tip_marker_show+np.array([0, 50])), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                        # cv2.putText(img_dst, f'[{1000*left_tip_1[0]:.1f}, {1000*left_tip_1[1]:.1f}, {1000*left_tip_1[2]:.1f}]',
                        #             tuple(np.array([660, 100], dtype=np.int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                        # left_f.write(f'{left_tip_pos_gravity[0]:.4f} {left_tip_pos_gravity[1]:.4f} {left_tip_pos_gravity[2]:.4f}\n')
                    if right_point_3d_gravity is not None:
                        right_tip_marker_show = right_tip_marker_show.astype(np.int32)
                        img_dst = cv2.circle(img_dst, tuple(right_tip_marker_show), 4, (0, 0, 255), -1)
                        right_tip_pos_gravity = GetTipPos(right_point_3d_gravity, right_rcm_pos[0:3], T_0_c)
                        right_tip_1 = beta*right_tip_0 + (1-beta)*right_tip_pos_gravity
                        right_tip_0 = right_tip_1
                        right_tip = right_tip_1.astype(np.float32)
                        right_pub.publish(right_tip)
                        # cv2.putText(img_dst, f'[{1000*right_tip_pos_gravity[0]:.1f}, {1000*right_tip_pos_gravity[1]:.1f}, {1000*right_tip_pos_gravity[2]:.1f}]',
                        #             tuple(right_tip_marker_show+np.array([0,50])), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                        # cv2.putText(img_dst, f'[{1000*right_tip_1[0]:.1f}, {1000*right_tip_1[1]:.1f}, {1000*right_tip_1[2]:.1f}]',
                        #             tuple(np.array([1260, 100],dtype=np.int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                        # right_f.write(f'{right_tip_pos_gravity[0]:.4f} {right_tip_pos_gravity[1]:.4f} {right_tip_pos_gravity[2]:.4f}\n')
                    start4 = time.time()
                    cv2.imshow('figure', img_dst)
                    # outCamera.write(img_show)
                    cv2.waitKey(1)
                    # print(time.time()-start4)
                print(f'total:{time.time() - start0}')


    except KeyboardInterrupt:
        cv2.destroyWindow('figure')
        capture.release()
        # left_f.close()
        # right_f.close()
