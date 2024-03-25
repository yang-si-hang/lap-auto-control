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

from geometry_msgs.msg import PoseStamped

import pandas as pd
from spatialmath.base import *

import sys
import os
sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set

time_start = None
file_name = lap_set.qtm_rigid_file_path


# calibration_rigid_msg = PoseStamped()
# calibration_rigid_name = "calibration_rigid"
msg_finished_flag = False

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
            # Extract one specific body
            rigid_index = body_index[rigid_name]
            position, rotation = bodies[rigid_index]
            
            R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
            

            if not np.isnan(position[0]):  
                # print(f"{rigid_name} \n{R}\n{position}")
                print(f"{rigid_name}")

            q = r2q(R)
            # print("q: ",q)

            '''     timestamp、     i、             name、          xyz、                                                      wxyz、'''
            data = f'{time_stamp},\t{rigid_index},\t{rigid_name},\t{position[0]/1000},{position[1]/1000},{position[2]/1000},\t{q[0]},{q[1]},{q[2]},{q[3]}\n'
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
    file = open(file_name, 'a')
    file.truncate(0)

    # rospy.init_node('qualisys_rigids_record', anonymous=True)

    

    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
