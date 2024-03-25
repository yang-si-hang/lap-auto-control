import cupy as cp
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

x_range = cp.arange(0, 1921, 1)
y_range = cp.arange(0, 1081, 1)
'''
f_0 是一个尺度参数，影响整个函数的幅度。
f_1 是旋转角度参数，表示二维高斯的旋转。
f_2 和 f_3 是两个标准差参数，控制高斯分布的形状。
f_4 和 f_5 是二维高斯分布的中心坐标。
f_6 是一个常数偏移项。
'''
'''                      幅度，       旋转角度，   x标准差,        y标准差,       x中心,       y中心,         常数偏移  '''
fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319]
fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680]  

# 使用向量化操作计算函数值
X, Y = cp.meshgrid(x_range, y_range)
J_left = (
    fun_left_0[6] + fun_left_0[0] * cp.exp(
        -(((X - fun_left_0[4]) * cp.cos(fun_left_0[1] * cp.pi / 180) 
           + (Y - fun_left_0[5]) * cp.sin(fun_left_0[1] * cp.pi / 180)) / fun_left_0[2]) ** 2
        - ((-(X - fun_left_0[4]) * cp.sin(fun_left_0[1] * cp.pi / 180) 
            + (Y - fun_left_0[5]) * cp.cos(fun_left_0[1] * cp.pi / 180)) / fun_left_0[3]) ** 2
    )
)

J_right = (
    fun_right_0[6]
    + fun_right_0[0] * cp.exp(
        -(((X - fun_right_0[4]) * cp.cos(fun_right_0[1] * cp.pi / 180) + (Y - fun_right_0[5]) * cp.sin(fun_right_0[1] * cp.pi / 180)) / fun_right_0[2]) ** 2
        - ((-(X - fun_right_0[4]) * cp.sin(fun_right_0[1] * cp.pi / 180) + (Y - fun_right_0[5]) * cp.cos(fun_right_0[1] * cp.pi / 180)) / fun_right_0[3]) ** 2
    )
)

function_values = J_left + J_right

# 将数据传回 CPU 以便绘图
function_values_cpu = cp.asnumpy(function_values)

# 展平 X 和 Y
X_flat = cp.asnumpy(X).flatten()
Y_flat = cp.asnumpy(Y).flatten()

# 调整字体大小并设置中文字体
font_size = 12
font_properties = FontProperties(fname='/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', size=font_size)

# 可视化函数
#处理中文乱码
# plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei']

plt.imshow(function_values_cpu, extent=(X_flat.min(), X_flat.max(), Y_flat.max(), Y_flat.min()), cmap='viridis', origin='lower', aspect='auto')

cbar = plt.colorbar(label='probability')

plt.xlabel('X')
plt.ylabel('Y')
# plt.title('手术器械分布律', fontsize=font_size, fontproperties=font_properties)
plt.show()




# =====================================================================================================================
# ============================================          CPU           =================================================
# =====================================================================================================================

# import numpy as np
# import matplotlib.pyplot as plt

# def calculate_function_value(x, y, fun_params):
#     J1 = fun_params[6] + fun_params[0] * np.exp(
#         -(((x - fun_params[4]) * np.cos(fun_params[1] * np.pi / 180) + (
#                 y - fun_params[5]) * np.sin(fun_params[1] * np.pi / 180)) / fun_params[2]) ** 2 \
#         - ((-(x - fun_params[4]) * np.sin(fun_params[1] * np.pi / 180) + (
#                 y - fun_params[5]) * np.cos(fun_params[1] * np.pi / 180)) / fun_params[3]) ** 2)
    
#     return J1

# # 设置像素范围
# x_range = np.arange(0, 1921, 1)  # 步长为1
# y_range = np.arange(0, 1081, 1)  # 步长为1

# # 初始化函数参数，这里你需要提供具体的参数值
# '''
# f_0 是一个尺度参数，影响整个函数的幅度。
# f_1 是旋转角度参数，表示二维高斯的旋转。
# f_2 和 f_3 是两个标准差参数，控制高斯分布的形状。
# f_4 和 f_5 是二维高斯分布的中心坐标。
# f_6 是一个常数偏移项。
# '''
# '''            幅度，       旋转角度，         x标准差,        y标准差,       x中心,       y中心,         常数偏移  '''
# fun_left_0 = [0.05410334, 0, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 0, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680]  

# # 计算函数值
# function_values = np.zeros((len(x_range), len(y_range)))
# for i, x in enumerate(x_range):
#     for j, y in enumerate(y_range):
#         function_values[i, j] = calculate_function_value(x, y, fun_left_0) + calculate_function_value(x, y, fun_right_0)

# # 可视化
# X, Y = np.meshgrid(x_range, y_range)
# plt.pcolormesh(X, Y, function_values.T, cmap='viridis')
# plt.colorbar(label='Function Value')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Function Visualization')
# plt.show()
