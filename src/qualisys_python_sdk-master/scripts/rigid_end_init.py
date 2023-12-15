"""
    Streaming 6Dof from QTM
    文件路径、ip、密码 在lap_set中修改
    该程序用于术前测量、设定、刚体到器械末端的变换矩阵
    
    注意：除了main，main中的 def on_packet(packet): 也要用global
"""

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources

import qtm_rt

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import time

from geometry_msgs.msg import PoseStamped

import pandas as pd
from spatialmath.base import *

import sys
import os
sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set

time_start = None



# calibration_rigid_msg = PoseStamped()
# calibration_rigid_name = "calibration_rigid"
msg_finished_flag = False

calibrating_rigid_name = ''
record_finish = True
rigid_end_file_path = lap_set.rigid_end_calibration_file_path


QTM_FILE = pkg_resources.resource_filename("qtm_rt", "data/Demo.qtm")


def matrix2quaternion(m):
    #m:array
    w = ((np.trace(m) + 1) ** 0.5) / 2
    x = (m[2][1] - m[1][2]) / (4 * w)
    y = (m[0][2] - m[2][0]) / (4 * w)
    z = (m[1][0] - m[0][1]) / (4 * w)
    return np.array([w,x,y,z])

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)
    # xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

def body_enabled_count(xml_string):
    xml = ET.fromstring(xml_string)
    return sum(enabled.text == "true" for enabled in xml.findall("*/Body/Enabled"))

async def main():
    global calibration_rigid_name
    global calibration_rigid_msg
    global msg_finished_flag
    global calibrating_rigid_name , record_finish
    # Connect to qtm
    # connection = await qtm_rt.connect("127.0.0.1")
    connection = await qtm_rt.connect(lap_set.qualisys_master_ip)

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    # async with qtm_rt.TakeControl(connection, "password"):
    async with qtm_rt.TakeControl(connection, lap_set.qualisys_password):

        # realtime = False  #????
        realtime = True

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            await connection.load(QTM_FILE)

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    print("{} of {} 6DoF bodies enabled".format(body_enabled_count(xml_string), len(body_index)))

    # wanted_body = "L-frame"
    # wanted_body = "umbrella"
    # wanted_body1 = "UR_L_Li" 

    def on_packet(packet):

        global calibrating_rigid_name , record_finish

        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )
        time_stamp = time.time()
        record_time_length = time_stamp - time_start
        # print(f"\n{time_stamp}")
        # print(f'录制时长：{int(record_time_length//3600):d}时 {(int(record_time_length)%3600)//60:d}分 {int(record_time_length%60):d}秒')
        for rigid_name in body_index:
            # Extract one specific body
            rigid_index = body_index[rigid_name]
            position, rotation = bodies[rigid_index]

            R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
            # print(f"{rigid_name} \n",R)
            # print(f"{rigid_name}")

            q = r2q(R)
            # print("q: ",q)

            # timestamp、i、name、xyz、wxyz、
            data = f'{time_stamp},\t{rigid_index},\t{rigid_name},\t{position[0]/1000},{position[1]/1000},{position[2]/1000},\t{q[0]},{q[1]},{q[2]},{q[3]}\n'
            

            # file.write(data)
            
            # print(f'{(time.time()-time_start):.6f} s')
            # print("{} - Pos: {} - Rot: {}".format(calibration_rigid_name, position, rotation))

            if rigid_name == calibrating_rigid_name and not record_finish :
                # print(f'data of tracking: \n{data}\n')
                # print(f'R:\n{R}\n{[[position[0]],[position[1]],[position[2]]]}')
                T_qualisys_rigid = np.concatenate((R.squeeze(), np.array([[position[0]/1000,position[1]/1000,position[2]/1000]]).T), axis=1)
                T_qualisys_rigid = np.concatenate((T_qualisys_rigid, np.array([[0,0,0,1]])), axis=0)
                print(f'T_qualisys_rigid {rigid_name}:\n{T_qualisys_rigid}\n')

                T_rigid_qualisys = np.linalg.inv(T_qualisys_rigid)
                # T_rigid_qualisys = np.concatenate((R.squeeze().T, np.array([[position[0]/1000,position[1]/1000,position[2]/1000]]).T*-1), axis=1)
                # T_rigid_qualisys = np.concatenate((T_rigid_qualisys, np.array([[0,0,0,1]])), axis=0)
                print(f'T_rigid_qualisys {rigid_name}:\n{T_rigid_qualisys}\n')
                print(T_qualisys_rigid @ T_rigid_qualisys)


                if np.isnan(position[0]):
                    record_finish = True
                    break


                usr_input = input("请选择末端球位置模式：\n1：单球模式\t2:双球模式\t回车：跳过\n")
                if usr_input == '1':

                    while  True:
                        temp = []
                        temp.append(float(input('请输入x：')))
                        temp.append(float(input('请输入y：')))
                        temp.append(float(input('请输入z：')))
                        usr_input = input(f'末端标记球坐标为：{temp}\n确认无误请按回车...')
                        if usr_input == '':
                            break

                    p_qualisys_end = np.array(temp)

                elif usr_input == '2':
                    while  True:
                        temp1 = []
                        temp1.append(float(input('请输入x1：')))
                        temp1.append(float(input('请输入y1：')))
                        temp1.append(float(input('请输入z1：')))
                        usr_input = input(f'末端标记球1坐标为：{temp1}\n确认无误请按回车...')
                        if usr_input == '':
                            break
                    while  True:
                        temp2 = []
                        temp2.append(float(input('请输入x2：')))
                        temp2.append(float(input('请输入y2：')))
                        temp2.append(float(input('请输入z2：')))
                        usr_input = input(f'末端标记球2坐标为：{temp2}\n确认无误请按回车...')
                        if usr_input == '':
                            break
                    p_qualisys_end = (np.array(temp1) + np.array(temp2))/2

                else:
                    p_qualisys_end = np.array([0,0,0]) 

                p_qualisys_end = np.append(p_qualisys_end, np.array([1]))
                print(f'末端标记球位置:{p_qualisys_end}')
                P_rigid_end = T_rigid_qualisys @ p_qualisys_end

                usr_input = input("请输入偏置量x：(无偏置请回车)")
                if usr_input == '':
                    p_adjust = np.array([0,0,0,0])
                else:
                    p_adjust = []
                    p_adjust.append(float(usr_input))
                    p_adjust.append(float(input("请输入偏置量y：")))
                    p_adjust.append(float(input("请输入偏置量z：")))
                    p_adjust.append(0)
                    p_adjust = np.array(p_adjust)
                    print(f'p_adjust: {p_adjust}')
                
                P_rigid_end = P_rigid_end + p_adjust
                print(f'末端相对刚体位置：{P_rigid_end}')
                    
                    

                rigid_end_file = open(rigid_end_file_path + str(calibrating_rigid_name) + '.txt', 'a')
                rigid_end_file.truncate(0)
                np.savetxt(rigid_end_file, P_rigid_end, delimiter=",")
                rigid_end_file.close()
                record_finish = True





    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    while True:
        # Wait asynchronously seconds
        if record_finish:
            if calibrating_rigid_name != '':
                print(f'{calibrating_rigid_name}已标定完成')
                

            usr_input = input('请输入校准刚体的名称(不更改请按回车)：')

            if not usr_input == '':
                calibrating_rigid_name = usr_input
                record_finish = False

        await asyncio.sleep(0.2)
        pass



    # Stop streaming

    tracking_file.close()
    await connection.stream_frames_stop()


if __name__ == "__main__":
    time_start = time.time()



    # rospy.init_node('qualisys_rigids_record', anonymous=True)

    

    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
