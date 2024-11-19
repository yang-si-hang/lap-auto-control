'''
用于测试相机标定结果（眼在手上）
需要使用棋盘格
路径仅依赖于lap_set
是否对原本的变换坐标进行修正：T_rob_cam_adjust_flag
'''

import os.path
import cv2
import numpy as np
np.set_printoptions(precision=8,suppress=True)
import glob
import math
from sklearn.decomposition import PCA
import sys
import rospy
import time
from scipy.spatial.transform import Rotation as R



sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set
#===================================================================================================

T_rob_cam_adjust_flag = True
joints = np.array(
    [
        [],
        []
    ]
)

# 角点的个数以及棋盘格间距 =================================================
# 小标定板----------------------------------------
# XX = 8
# YY = 11
XX = 11
YY = 8
L = 0.015
black_background = False #黑色背景为true（黑色外圈的），若果为黑色背景后续会转换为白色底
board_corner_matrix = np.zeros([YY,XX,4])
base_corner_matrix = np.zeros([YY,XX,4])
for i in range(YY):
    for j in range(XX):
        board_corner_matrix[i,j] = np.array([j-1, i-1, 0, 0]) * L
        board_corner_matrix[i,j,3] = 1
# print(board_corner_matrix)
# =======================================================================

folder = f'{lap_set.data_folder}/Camera_Calibration'
mtx_path = f'{folder}/mtx.csv' #内参矩阵
dist_path=f'{folder}/dist.csv'
camera_tool_path = f'{folder}/camera_tool.csv'
T_0_rob_catted_path = f'{folder}/RobotPose.csv'
#读取矩阵文件，写入格式问题，手动逐行处理
T_0_rob_catted = []
f=open(T_0_rob_catted_path)
lines=f.readlines() #将全部数据读到一个lines中
for line in lines:
    list=line.strip('\n').split(',')
    T_0_rob_catted.append(list[0:4])
T_0_rob_catted = np.array(T_0_rob_catted).astype(float)
# print(T_0_rob_catted)

mtx = np.loadtxt(mtx_path) #内参矩阵

temp_matrix = np.loadtxt(camera_tool_path) #相机相对于机械臂末端的变换矩阵
if T_rob_cam_adjust_flag:
# 定义绕 z 轴旋转 90 度
    rotation_matrix = R.from_euler('z', -90, degrees=True).as_matrix()
    print(f'rotation matrix :\n{rotation_matrix}')
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    T_rob_cam = transformation_matrix @ temp_matrix
else:
    T_rob_cam = temp_matrix

# T_rob_cam = np.eye(4)
# T_rob_cam[:3,0] = -temp_matrix[:3,1]
# T_rob_cam[:3,1] = temp_matrix[:3,0]
# T_rob_cam[:3,2] = temp_matrix[:3,2]
# T_rob_cam[:3,3] = temp_matrix[:3,3]
print(f'T_rob_cam :\n{T_rob_cam}')
fx = mtx[0,0]
fy = mtx[1,1]
cx = mtx[0,2]
cy = mtx[1,2]


images = glob.glob(f'{lap_set.data_folder}/Camera_Calibration/imgs/*.png')
images_path = sorted(images)  #按照文件名排序，数字的位数不同会乱，可以使用 str（int）.zfill（3）将整数前补0化为固定位数 001
print(f'图片路径：{images_path[0]}\n图片数量：{len(images_path)}')
print(f'T_0_rob 个数:{int(len(T_0_rob_catted)/4)}')

T_0_board = np.eye(4)



def T_cam_board_calculate(img,XX,YY):
    '''
    根据拍有棋盘格的图片和相机内参，估算当前相机坐标系下标定板的坐标。
    标定板的原点选为能检测到的角点的第二行第二列，因为最外侧的z轴估计可能不如内圈的准确，因为内圈用了四个估计值求平均。
    x、y、z轴正方向均采用和相机坐标系基本相同。

    args:
        img: 图片，分辨率与标定时的一致才行
        XX、YY: 棋盘格xy方向角点数量
    return:
        T_cam_board

    ps:内含棋盘格角点识别的相关画图，如需要可开启
    '''
    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = L*objp

    obj_points = []     # 存储3D点
    img_points = []     # 存储2D点

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if black_background == True:
        gray = 255-gray #一定要转换成白色为底
    # cv2.namedWindow("img_gray", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("img_gray", 400, 300)
    # cv2.imshow('img_gray',gray)
    # key = cv2.waitKey(10) #需要一定演示才能显示，单位ms

    ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
    corners = corners.squeeze()

    if ret:

        obj_points.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点，（2024.5.23：目前测试corners2和corner计算结果毫无区别）
        print('corners2:',corners2.shape)
        if [corners2]:
            img_points.append(corners2)
        else:
            img_points.append(corners)

        # cv2.drawChessboardCorners(img, (XX, YY), corners, ret)
        # 红色为第一行点，蓝色为最后一行点，数点先X轴再Y轴，一行有XX个点
        # cv2.imwrite(f'{os.path.dirname(__file__)}/../data/Camera_Calibration/figure_test/{i}.jpg', img)
        
        # cv2.imshow('img', img)
        # cv2.waitKey(2000)

        corners2 = corners2.squeeze()
        corner_matrix = corners2.reshape([YY,XX,2])
        # print(corner_matrix[0])
        z_matrix = np.zeros([YY,XX])#用于保存各角点z值估计值
        cam_corner_matrix = np.zeros([YY,XX,3])
        for j in range(YY):
            for k in range(XX):
                z_list = [] #用于保存估计的标定板到镜头的距离
                if k != 0:
                    delta_u = corner_matrix[j,k,0] - corner_matrix[j,k-1,0]
                    delta_v = corner_matrix[j,k,1] - corner_matrix[j,k-1,1]
                    z = math.sqrt(L*L/(pow(delta_u/fx,2)+pow(delta_v/fy,2)))
                    z_list.append(z)
                if k != XX-1:
                    delta_u = corner_matrix[j,k,0] - corner_matrix[j,k+1,0]
                    delta_v = corner_matrix[j,k,1] - corner_matrix[j,k+1,1]
                    z = math.sqrt(L*L/(pow(delta_u/fx,2)+pow(delta_v/fy,2)))
                    z_list.append(z)
                if j != 0:
                    delta_u = corner_matrix[j,k,0] - corner_matrix[j-1,k,0]
                    delta_v = corner_matrix[j,k,1] - corner_matrix[j-1,k,1]
                    z = math.sqrt(L*L/(pow(delta_u/fx,2)+pow(delta_v/fy,2)))
                    z_list.append(z)
                if j != YY-1:
                    delta_u = corner_matrix[j,k,0] - corner_matrix[j+1,k,0]
                    delta_v = corner_matrix[j,k,1] - corner_matrix[j+1,k,1]
                    z = math.sqrt(L*L/(pow(delta_u/fx,2)+pow(delta_v/fy,2)))
                    z_list.append(z)
                z = sum(z_list)/len(z_list)
                z_matrix[j,k] = z
                cam_corner_matrix[j,k] = np.array([(corner_matrix[j,k,0]-cx)*z/fx, (corner_matrix[j,k,1]-cy)*z/fy, z])


                # 定义文本及其属性
                text = f'{z:.3f}'
                position = (tuple(corner_matrix[j,k].astype(int)))  # 文本的左下角位置，(x, y) 坐标
                font = cv2.FONT_HERSHEY_SIMPLEX  # 字体类型
                font_scale = 0.25  # 字体大小
                color = (255, 0, 0)  # 颜色，白色 (B, G, R)
                thickness = 1  # 字体线条粗细
                line_type = cv2.LINE_AA  # 线条类型

                # 在图像上添加文本
                cv2.putText(img, text, position, font, font_scale, color, thickness, line_type)
                cv2.circle(img, position, radius=1, color=(0,0,255), thickness=-1)


        # cv2.imwrite(f'{os.path.dirname(__file__)}/../data/Camera_Calibration/figure_test/{i}.jpg', img)
        z_max = np.max(z_matrix)
        z_min = np.min(z_matrix)
        z_mean = np.mean(z_matrix)
        print(f'z_matrix: 长度：{z_matrix.size}(应为{(XX)*(YY)})  均值：{z_mean:.4f}  极差：{z_max-z_min:.4f}')


        # #只取起点、终点两点向量来计算x、y轴（弃用）
        # cam_board_x = cam_corner_matrix[1,XX-2] - cam_corner_matrix[1,1]
        # cam_board_y = cam_corner_matrix[YY-2,1] - cam_corner_matrix[1,1]
        # print('=============')
        # print(cam_board_x)
        # print(cam_board_y)

        # 通过直线拟合来计算x，y轴向量
        # 执行PCA
        pca = PCA(n_components=2)
        pca.fit(cam_corner_matrix[1,1:XX-1,:])
        # 获取主成分（方向向量）
        pca_x = pca.components_[0]
        if pca_x[0]<0:
            pca_x = -pca_x
        # print("方向向量:", pca_x * np.linalg.norm(cam_board_x))
        
        pca.fit(cam_corner_matrix[1:YY-1,1,:])
        # 获取主成分（方向向量）
        pca_y = pca.components_[0]
        if pca_y[1]< 0:
            pca_y = -pca_y
        # print("方向向量:", pca_y * np.linalg.norm(cam_board_y))
        pca_z = np.cross(pca_x, pca_y)
        pca_y = np.cross(pca_z, pca_x)
        pca_y = pca_y/np.linalg.norm(pca_y)

        _T_cam_board = np.eye(4)
        _T_cam_board[:3,0] = pca_x
        _T_cam_board[:3,1] = pca_y
        _T_cam_board[:3,2] = pca_z
        _T_cam_board[:3,3] = cam_corner_matrix[1,1]
        print(f'T_cam_board:\n{_T_cam_board}')
        return _T_cam_board


    


if __name__=="__main__":
    # rospy.init_node('camera_calibration_test')
    # rokae = rokae_basic_fun.rokae()
    # rokae.jp_cmd(joints[0])
    # time.sleep(0.5)

    # # 打开USB摄像头
    # capture = cv2.VideoCapture(0)
    # capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('N', 'V', '1', '2'))
    # capture.set(cv2.CAP_PROP_FPS, lap_set.video_fps)
    # capture.set(4, lap_set.video_height)  # 图片高度
    # capture.set(3, lap_set.video_width)  # 图片宽度

    # # 检查摄像头是否成功打开
    # if not capture.isOpened():
    #     print("无法打开摄像头")
    #     exit()

    # # 获取摄像头的帧宽度和帧高度
    # frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    # frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # ret, frame = capture.read()
    # while not ret:
    #     ret, frame = capture.read()





    img = cv2.imread(images_path[0])
    T_cam_board = T_cam_board_calculate(img, XX, YY)
    T_0_board = T_0_rob_catted[:4,:] @ T_rob_cam @ T_cam_board
    print(f'T_0_board:\n{T_0_board}')
    
    for i in range(YY):
        for j in range(XX):
            base_corner_matrix[i,j] = T_0_board @ board_corner_matrix[i,j]

    
    for i in range(0,len(images_path)):
    # for i in range(0,1):
        print(f'------ {i} --------')
        cam_corner_matrix = np.zeros([YY,XX,4])
        img_corner_matrix = np.zeros([YY,XX,2])
        img = cv2.imread(images_path[i])
        T_0_rob = T_0_rob_catted[4*i:4*i+4,:]
        T_0_cam = T_0_rob @ T_rob_cam
        print(f'T_0_cam:\n{T_0_cam}')
        T_cam_0 = np.linalg.inv(T_0_cam)
        for row in range(YY):
            for vol in range(XX):
                cam_corner_matrix[row,vol] = T_cam_0 @ base_corner_matrix[row,vol]
                img_corner_matrix[row,vol] = (mtx @ (cam_corner_matrix[row,vol,:3]/cam_corner_matrix[row,vol,2]))[:2]
                temp_tuple = img_corner_matrix[row,vol].astype(int)
                # print(temp_tuple)
                cv2.circle(img, temp_tuple, radius=3, color=(0,0,255), thickness=-1)
        flag = cv2.imwrite(f'{lap_set.data_folder}/Camera_Calibration/figure_check/{i}.png', img)
        print(f'figure_check {i} saved: {flag}')



    
        
    
    # rokae.jp_cmd(joints[2])
    # time.sleep(1)






        

