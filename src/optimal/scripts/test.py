import os
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random



import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
sys.path.append(f'{os.path.dirname(__file__)}/../src/optimal/scripts')
from lap_set_pk import lap_set

def intersection_of_multi_lines(strt_points, directions):  
    '''
    strt_points: line start points; numpy array, nxdim
    directions: list dierctions; numpy array, nxdim  

    return: the nearest points to n lines 
    '''
    strt_points = np.array(strt_points)
    directions = np.array(directions)
    n, dim = strt_points.shape

    G_left = np.tile(np.eye(dim), (n, 1))  
    G_right = np.zeros((dim*n, n))  

    for i in range(n):
        G_right[i*dim:(i+1)*dim, i] = -directions[i, :]

    G = np.concatenate([G_left, G_right], axis=1)  
    d = strt_points.reshape((-1, 1)) 

    m = np.linalg.inv(np.dot(G.T, G)).dot(G.T).dot(d)   

    return m[0:dim].squeeze()
    # return m


# # 随机生成一个共同焦点
# np.random.seed(42)  # 设置随机种子，确保结果可重现
# common_focus = np.random.uniform(-30,30,3)  # 生成随机的交点
# print("共同焦点的坐标:", common_focus)

# # 生成5条直线，所有直线的起点和方向带有一定随机性，且都指向共同焦点
# num_lines = 5
# points = []
# directions = []

# for i in range(num_lines):
#     # 为每条直线生成一个随机起点，范围[0, 10]
#     p = np.random.uniform(-100, 100, size=3)

#     # 计算从起点指向共同焦点的方向向量
#     d = common_focus+np.random.uniform(-5, 5, size=3)  - p
#     d = random.choice([-1, 1]) * d
#     d = d / np.linalg.norm(d)  # 归一化方向向量

#     points.append(p)
#     directions.append(d)

# # 输出生成的点和方向
# print("\n生成的直线上的点：")
# print(np.array(points))
# print("\n生成的直线方向向量：")
# print(np.array(directions))

# # 计算最佳交点
# common_point = intersection_of_multi_lines(points, directions)
# print("\n最佳交点坐标:", common_point)



# # 可视化
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# # 绘制每条直线
# t_vals = np.linspace(-200, 200, 100)
# for i in range(num_lines):
#     p = points[i]
#     d = directions[i]
#     line_points = p + t_vals[:, None] * d  # 计算直线上的点
#     ax.plot(line_points[:, 0], line_points[:, 1], line_points[:, 2], label=f"Line {i+1}")

# # 绘制交点
# ax.scatter(common_point[0], common_point[1], common_point[2], color='r', s=100, label='Intersection Point')

# # 设置图像标签
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # 添加图例
# ax.legend()

# # 显示图像
# plt.show()


lap_shaft_path = f"{lap_set.data_folder}/pre_data/lap_shaft.txt"# 用于保存{base}下的 腹腔镜轴线（根据几何尺寸推算）
lap_shaft_arrray = np.loadtxt(lap_shaft_path,delimiter=',')
strt_points = lap_shaft_arrray[:,:3]
directions = lap_shaft_arrray[:,3:]
lap_intersection = intersection_of_multi_lines(strt_points, directions)
# lap_intersection = lap_set.intersection_of_multi_lines(input_file=lap_shaft_path)


# 可视化
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制每条直线
t_vals = np.linspace(-200, 200, 100)
for i in range(strt_points.shape[0]):
    p = strt_points[i]
    d = directions[i]
    line_points = p + t_vals[:, None] * d  # 计算直线上的点
    ax.plot(line_points[:, 0], line_points[:, 1], line_points[:, 2], label=f"Line {i+1}")

# 绘制交点
ax.scatter(lap_intersection[0], lap_intersection[1], lap_intersection[2], color='r', s=100, label='Intersection Point')

# 设置图像标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 添加图例
ax.legend()

#坐标范围
offset = 0.2  # 0.2米
ax.set_xlim(lap_intersection[0] - offset, lap_intersection[0] + offset)
ax.set_ylim(lap_intersection[1] - offset, lap_intersection[1] + offset)
ax.set_zlim(lap_intersection[2] - offset, lap_intersection[2] + offset)

# 显示图像
plt.show()


