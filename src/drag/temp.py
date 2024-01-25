import numpy as np

l =[]
for i in range(10):
    l.append([1,2,3,4,5])
n = np.array(l)
n2 = n.mean(axis=0)
print(n)
print(n2)

import numpy as np

# 创建两个矩阵
matrix1 = np.array([[1, 2], [3, 4]])
matrix2 = np.array([[5, 6]])

# 垂直拼接两个矩阵
result_vertical = np.concatenate((matrix1, matrix2), axis=0)

# 打印结果
print("垂直拼接结果:")
print(result_vertical)

# 创建两个矩阵
matrix3 = np.array([[7], [8]])
matrix4 = np.array([[9], [10]])

# 水平拼接两个矩阵
result_horizontal = np.concatenate((matrix3, matrix4), axis=1)

# 打印结果
print("\n水平拼接结果:")
print(result_horizontal)



from math3d.transform import Transform as Trans

T_rob_sensor = np.array([   [0,    1,  0,   0],
                                    [0,    0,  -1,  0],
                                    [-1,   0,  0,   0],
                                    [0,    0,  0,   1]])

T = Trans(T_rob_sensor)

print(T.array)
print(T.pose_vector)



import os

# 获取当前脚本所在的文件夹路径
current_folder = os.path.dirname(__file__)

print("当前文件夹路径:", current_folder)


list1 = [1,2,3]
list2 = [4,5,6]
print(list1 + list2)
print([list1, list2])




