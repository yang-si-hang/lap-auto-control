import cupy as cp
import matplotlib.pyplot as plt

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
fun_left_0 = cp.array([0.05410334,  0,          377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319])
fun_right_0 = cp.array([0.07491880, 50,          99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680])

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



# 可视化函数
plt.imshow(function_values_cpu, extent=(X_flat.min(), X_flat.max(), Y_flat.max(), Y_flat.min()), cmap='viridis', origin='lower', aspect='auto')
plt.colorbar(label='Function Value')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Function Visualization')
plt.show()