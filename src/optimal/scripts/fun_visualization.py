import numpy as np
import matplotlib.pyplot as plt

def calculate_function_value(x, y, fun_params):
    J1 = fun_params[6] + fun_params[0] * np.exp(
        -(((x - fun_params[4]) * np.cos(fun_params[1] * np.pi / 180) + (
                y - fun_params[5]) * np.sin(fun_params[1] * np.pi / 180)) / fun_params[2]) ** 2 \
        - ((-(x - fun_params[4]) * np.sin(fun_params[1] * np.pi / 180) + (
                y - fun_params[5]) * np.cos(fun_params[1] * np.pi / 180)) / fun_params[3]) ** 2)
    
    return J1

# 设置像素范围
x_range = np.arange(0, 1921, 1)  # 步长为1
y_range = np.arange(0, 1081, 1)  # 步长为1

# 初始化函数参数，这里你需要提供具体的参数值
'''
f_0 是一个尺度参数，影响整个函数的幅度。
f_1 是旋转角度参数，表示二维高斯的旋转。
f_2 和 f_3 是两个标准差参数，控制高斯分布的形状。
f_4 和 f_5 是二维高斯分布的中心坐标。
f_6 是一个常数偏移项。
'''
'''            幅度，       旋转角度，         x标准差,        y标准差,       x中心,       y中心,         常数偏移  '''
fun_left_0 = [0.05410334, 0, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319]
fun_right_0 = [0.07491880, 0, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680]  

# 计算函数值
function_values = np.zeros((len(x_range), len(y_range)))
for i, x in enumerate(x_range):
    for j, y in enumerate(y_range):
        function_values[i, j] = calculate_function_value(x, y, fun_left_0) + calculate_function_value(x, y, fun_right_0)

# 可视化
X, Y = np.meshgrid(x_range, y_range)
plt.pcolormesh(X, Y, function_values.T, cmap='viridis')
plt.colorbar(label='Function Value')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Function Visualization')
plt.show()


# ================================================
# import cupy as cp
# import matplotlib.pyplot as plt

# # # 将参数和范围转换为 CuPy 数组
# x_range = cp.arange(0, 1921, 1)  # 步长为10
# y_range = cp.arange(0, 1081, 1)  # 步长为10
# # fun_params = cp.array([1, 45, 2, 3, 100, 200, 0])
# fun_left_0 = cp.array([0.05410334, 0, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319])
# fun_right_0 = cp.array([0.07491880, 0, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680])  

# # 计算函数值的函数
# def calculate_function_value(x, y, fun_params):
#     J_left = (
#         fun_params[6]
#         + fun_params[0] * cp.exp(
#             -(((x - fun_params[4]) * cp.cos(fun_params[1] * cp.pi / 180) + (y - fun_params[5]) * cp.sin(fun_params[1] * cp.pi / 180)) / fun_params[2]) ** 2
#             - ((-(x - fun_params[4]) * cp.sin(fun_params[1] * cp.pi / 180) + (y - fun_params[5]) * cp.cos(fun_params[1] * cp.pi / 180)) / fun_params[3]) ** 2
#         )
#     )
#     return J_left

# # 在 GPU 上计算函数值
# function_values = cp.zeros((len(x_range), len(y_range)))
# for i, x in enumerate(x_range):
#     for j, y in enumerate(y_range):
#         function_values[i, j] = calculate_function_value(x, y, fun_left_0) + calculate_function_value(x, y, fun_right_0)

# # 将数据传回 CPU 以便绘图
# function_values_cpu = cp.asnumpy(function_values)

# # 可视化函数
# X, Y = cp.meshgrid(x_range, y_range)
# plt.pcolormesh(X, Y, function_values_cpu.T, cmap='viridis')
# plt.colorbar(label='函数值')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('函数可视化')
# plt.show()

