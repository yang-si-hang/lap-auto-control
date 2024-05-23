'''
用于测试相机标定结果（眼在手上）
需要使用棋盘格
'''
import json
import os.path
import cv2
import h5py
import numpy as np
np.set_printoptions(precision=8,suppress=True)
import glob

# 角点的个数以及棋盘格间距 =================================================
# 小标定板----------------------------------------
# XX = 8
# YY = 11
XX = 11
YY = 8
L = 0.015
black_background = False #黑色背景（黑色外圈的）后续转换为白色底
# =======================================================================



# 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

# 获取标定板角点的位置
objp = np.zeros((XX * YY, 3), np.float32)
objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
objp = L*objp

obj_points = []     # 存储3D点
img_points = []     # 存储2D点

images = glob.glob(f'{os.path.dirname(__file__)}/../data/Camera_Calibration/imgs/*.png')
images = sorted(images)  #按照文件名排序，数字的位数不同会乱，可以使用 str（int）.zfill（3）将整数前补0化为固定位数 001
print(f'图片路径：{images[0]}\n图片数量：{len(images)}')

exit()
# images = images[::-1]
i = 0
for fname in images:
    img = cv2.imread(fname)
    # cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("img", 400, 300)
    # cv2.imshow('img',img)
    # key = cv2.waitKey(10)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if black_background == True:
        gray = 255-gray #一定要转换成白色为底
    cv2.namedWindow("img_gray", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("img_gray", 400, 300)
    cv2.imshow('img_gray',gray)
    key = cv2.waitKey(10) #需要一定演示才能显示，单位ms

    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
    print(ret)

    if ret:

        obj_points.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
        #print(corners2)
        if [corners2]:
            img_points.append(corners2)
        else:
            img_points.append(corners)

        cv2.drawChessboardCorners(img, (XX, YY), corners, ret)
        # 红色为第一个点，蓝色为最后一个点，先X轴再Y轴
        cv2.imwrite(f'{os.path.dirname(__file__)}/../data/Camera_Calibration/figure_test/{i}.jpg', img)
        i = i+1
        # cv2.imshow('img', img)
        # cv2.waitKey(2000)

