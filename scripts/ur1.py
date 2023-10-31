import urx
import math3d as m3d
import numpy as np

import sys

sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

rob = urx.Robot(lap_set.robot_ip)
rob.set_tcp((0, 0, 0, 0, 0, 0)) 
rob.set_payload(2, (0, 0, 0.1))
# sleep(0.2)  #leave some time to robot to process the setup commands
print ("Current tool pose is: ",  rob.getl())
trans = rob.get_pose()
print('trans.array:\n',trans.array)
# tool=np.array([[0.0],[0.0],[0.08], [1]])
tool=np.array([[0.0],[0.0],[0.06], [1]])

tool_tip = np.squeeze(trans.array @ tool)
print(f'tool tip:\n[{tool_tip[0]}, {tool_tip[1]}, {tool_tip[2]}, {tool_tip[3]}]')


# trans.orient.rotate_xb(+np.pi/6)

# rob.set_pose(trans, acc=0.5, vel=0.2)
# print (trans)
# rob.movel((0.05, 0, 0, 0, 0, 0), 0.5, 0.1, relative=True) 
# rob.my_speedl([0, 0, 0.01, 0, 0, 0],0.5,3)
rob.close()