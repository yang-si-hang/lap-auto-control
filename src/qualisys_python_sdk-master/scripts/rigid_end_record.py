"""
    Streaming 6Dof from QTM
    文件路径、ip、密码 在lap_set中修改
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

time_start = None
rigid_end_record_file = lap_set.rigid_end_record_file_path
rigid_end_calibration_files = glob.glob(os.path.join(lap_set.rigid_end_calibration_file_path, '*'))
rigid_end_calibration_file_names  = [os.path.basename(file_path) for file_path in rigid_end_calibration_files]
rigid_names = [i.replace('.txt','') for i in rigid_end_calibration_file_names]


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

 

        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )
        time_stamp = time.time()
        record_time_length = time_stamp - time_start
        print(f"\n{time_stamp}")
        print(f'录制时长：{int(record_time_length//3600):d}时 {(int(record_time_length)%3600)//60:d}分 {int(record_time_length%60):d}秒')
        for rigid_name in body_index:
            if rigid_name in rigid_names:
                # Extract one specific body
                rigid_index = body_index[rigid_name]
                position, rotation = bodies[rigid_index]
                
                R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
                

                if not np.isnan(position[0]):  
                    T_qualisys_rigid = np.concatenate((R.squeeze(), np.array([[position[0]/1000,position[1]/1000,position[2]/1000]]).T), axis=1)
                    T_qualisys_rigid = np.concatenate((T_qualisys_rigid, np.array([[0,0,0,1]])), axis=0)
                    # print(f'{rigid_name}:\n{T_qualisys_rigid}')

                    # print(f"{rigid_name} \n{R}\n{position}")
                    p_rigid_end = np.loadtxt(lap_set.rigid_end_calibration_file_path + rigid_name + '.txt')
                    p_qalisys_end = T_qualisys_rigid @ p_rigid_end

                    print(f"{rigid_name}:{p_qalisys_end[0]},{p_qalisys_end[1]},{p_qalisys_end[2]}")

                else:
                    p_qalisys_end = [np.nan, np.nan, np.nan, 1]

                '''     timestamp、     i、             name、          xyz'''
                data = f'{time_stamp},\t{rigid_index},\t{rigid_name},\t{p_qalisys_end[0]},{p_qalisys_end[1]},{p_qalisys_end[2]}\n'
                file.write(data)
                # if not np.isnan(position[0]): print(f'{data}')
                # print(f'{(time.time()-time_start):.6f} s')
                # print("{} - Pos: {} - Rot: {}".format(calibration_rigid_name, position, rotation))
            


       

       

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    # while not rospy.is_shutdown():
    while True:
        # Wait asynchronously seconds
        await asyncio.sleep(1)
        pass



    # Stop streaming
    file.close()
    await connection.stream_frames_stop()


if __name__ == "__main__":
    time_start = time.time()
    file = open(rigid_end_record_file, 'a')
    file.truncate(0)


    # rospy.init_node('qualisys_rigids_record', anonymous=True)

    

    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
