"""
    Streaming 6Dof from QTM
    文件路径、ip、密码 在lap_set中修改

    会在以下位置保存
    # np.savetxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',rcm_pose,delimiter=',')
    # np.savetxt(f'{lap_set.coordinate_set_folder}/qualisys_rcm.txt', qualisys_rcm, delimiter=',')
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
import glob

from geometry_msgs.msg import PoseStamped

import pandas as pd
from spatialmath.base import *

import sys
import os
sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set
#================================================================================
lap_rigid = lap_set.rigid_names['rcm_calculate']
assert lap_rigid is not None, '刚体名未正常设置'


fold_path = f'{lap_set.data_folder}/Qualisys_calibration'
T_rob_rigid_path = f'{fold_path}/T_rob_rigid.txt'
T_0_qualisys_path = f'{fold_path}/T_0_cam.txt'


T_rob_rigid = np.loadtxt(T_rob_rigid_path,delimiter=',')
T_0_qualisys = np.loadtxt(T_0_qualisys_path,delimiter=',')
assert T_rob_rigid is not None, f"未能读取 T_rob_rigid : {T_rob_rigid_path}"
assert T_0_qualisys is not None, f"未能读取 T_0_qualisys : {T_0_qualisys_path}"
T_rigid_rob = np.linalg.inv(T_rob_rigid)



data_num = 0
time_start = None

start_points_list = []
directions_list = []


# print(p_rigids_ends)
# print(p_rigids_ends['temp'])


# calibration_rigid_msg = PoseStamped()
# calibration_rigid_name = "calibration_rigid"
msg_finished_flag = False

QTM_FILE = pkg_resources.resource_filename("qtm_rt", "data/Demo.qtm")



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
    global data_num
    global start_points_list, directions_list
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
        global data_num
 

        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )
        time_stamp = time.perf_counter()
        record_time_length = time_stamp - time_start
        print(f"\n{time_stamp}")
        print(f'运行时长：{int(record_time_length//3600):d}时 {(int(record_time_length)%3600)//60:d}分 {int(record_time_length%60):d}秒')
        for rigid_name in body_index:

            if rigid_name == lap_rigid:
                # Extract one specific body
                rigid_index = body_index[rigid_name]
                position, rotation = bodies[rigid_index]
                
                R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
                

                if not np.isnan(position[0]):  
                    T_qualisys_rigid = np.concatenate((R.squeeze(), np.array([[position[0]/1000,position[1]/1000,position[2]/1000]]).T), axis=1)
                    T_qualisys_rigid = np.concatenate((T_qualisys_rigid, np.array([[0,0,0,1]])), axis=0)
                    T_qualisys_rob = T_qualisys_rigid @ T_rigid_rob
                    start_points_list.append(np.squeeze(T_qualisys_rob[:3,3]).copy())
                    directions_list.append(np.squeeze(T_qualisys_rob[:3,2]).copy())


        data_num += 1


       

       

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    # while not rospy.is_shutdown():
    while True:
        # Wait asynchronously seconds
        await asyncio.sleep(1)
        pass






if __name__ == "__main__":
    time_start = time.perf_counter()


    # rospy.init_node('qualisys_rigids_record', anonymous=True)

    
    try:
        # Run our asynchronous function until complete
        asyncio.get_event_loop().run_until_complete(main())
        # asyncio.wait_for(main(),timeout= 3)
        # print('asyncio stop')
        # while True:
        #     pass

    except KeyboardInterrupt:
        print(f'record finished\ndata num:{data_num}')
        print(f'list lengths: {len(start_points_list)}  {len(directions_list)}')
    
    finally:
        qualisys_rcm = lap_set.intersection_of_multi_lines(start_points_list, directions_list)
        base0_rcm = T_0_qualisys @ np.append(qualisys_rcm,1)

        try:
            rcm_pose = np.loadtxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',delimiter=',')
        except:
            rcm_pose = np.loadtxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',delimiter=' ')

        rcm_pose[:3,3] = base0_rcm[:3]
        print(f'rcm:\n机械臂基坐标系: {base0_rcm[:3]}\nQualisys坐标系: {qualisys_rcm}')


        np.savetxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',rcm_pose,delimiter=',')
        np.savetxt(f'{lap_set.coordinate_set_folder}/qualisys_rcm.txt', qualisys_rcm, delimiter=',')


