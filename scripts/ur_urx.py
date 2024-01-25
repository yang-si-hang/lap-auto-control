import urx
import math3d as m3d
import numpy as np

import sys
import os

sys.path.append(f'{os.path.dirname(__file__)}/../src/optimal/scripts')
from lap_set_pk import lap_set

def matrix_with_dot(matrix):
    matrix_str = "["
    for i in range(4):
        for j in range(4):
            if j == 0:
                matrix_str += '['
            matrix_str += str(matrix[i, j])
            if j < 4 - 1:
                matrix_str += ", "
            elif i < 4 - 1:
                matrix_str += '],'
        if i < 4 - 1:
            matrix_str += "\n"
    matrix_str += ']]'
    # 打印结果
    # print(f'带分割符格式：\n{matrix_str}')
    return matrix_str

rob = urx.Robot(lap_set.robot_ip)
rob.set_tcp((0, 0, 0, 0, 0, 0)) 
rob.set_payload(2, (0, 0, 0.1))
# sleep(0.2)  #leave some time to robot to process the setup commands
print ("机械臂末端位姿:\n\t ",  rob.getl())
trans = rob.get_pose()

print('机械臂末端位姿:\n',matrix_with_dot(trans.array))

# tool=np.array([[0.0],[0.0],[0.08], [1]])
tool=np.array([[0.0],[0.0],[0.06], [1]])

tool_tip = np.squeeze(trans.array @ tool)
print(f'定位针末端位置:\n\t[{tool_tip[0]}, {tool_tip[1]}, {tool_tip[2]}, {tool_tip[3]}]')


# trans.orient.rotate_xb(+np.pi/6)

# rob.set_pose(trans, acc=0.5, vel=0.2)
# print (trans)
# rob.movel((0.05, 0, 0, 0, 0, 0), 0.5, 0.1, relative=True) 
# rob.my_speedl([0, 0, 0.01, 0, 0, 0],0.5,3)
rob.close()