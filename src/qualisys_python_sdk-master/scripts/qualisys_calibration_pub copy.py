"""
    Streaming 6Dof from QTM
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


time_start = None


calibration_rigid_msg = PoseStamped()
calibration_rigid_name = "calibration_rigid"
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
    connection = await qtm_rt.connect("192.168.254.1")

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    # async with qtm_rt.TakeControl(connection, "password"):
    async with qtm_rt.TakeControl(connection, "123456"):

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
        global calibration_rigid_name
        global calibration_rigid_msg
        global msg_finished_flag

        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )

        if calibration_rigid_name is not None and calibration_rigid_name in body_index:
            # Extract one specific body
            calibration_rigid_index = body_index[calibration_rigid_name]
            position, rotation = bodies[calibration_rigid_index]

            msg_finished_flag = False
            calibration_rigid_msg.pose.position.x = position[0] /1000.0
            calibration_rigid_msg.pose.position.y = position[1] /1000.0
            calibration_rigid_msg.pose.position.z = position[2] /1000.0
            print("p\n",position)
            R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达
            print("R\n",R)

            q = r2q(R)
            # print("q2: ",q)
            calibration_rigid_msg.pose.orientation.w = q[0]
            calibration_rigid_msg.pose.orientation.x = q[1]
            calibration_rigid_msg.pose.orientation.y = q[2]
            calibration_rigid_msg.pose.orientation.z = q[3]
            msg_finished_flag = True
            # print(calibration_rigid_msg)
            calibration_rigid_pub.publish(calibration_rigid_msg)
            # print(f'{(time.time()-time_start):.6f} s')


            
            # print("{} - Pos: {} - Rot: {}".format(calibration_rigid_name, position, rotation))


        else:
            # Print all bodies
            for position, rotation in bodies:
                print("Pos: {} - Rot: {}".format(position, rotation))

       

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    while not rospy.is_shutdown():
        # Wait asynchronously 5seconds
        await asyncio.sleep(1)
        # left_pub.publish(temp1.astype(np.float32))
        while not msg_finished_flag:
            pass
        if msg_finished_flag:
            
            # print("published")
            # print(calibration_rigid_msg)
            # print(f'{(time.time()-time_start):.6f} s')
            pass



    # Stop streaming
    await connection.stream_frames_stop()


if __name__ == "__main__":
    time_start = time.time()
    rospy.init_node('qualisys_calibration_pub', anonymous=True)
    calibration_rigid_pub = rospy.Publisher('calibration_rigid_pos', PoseStamped, queue_size=1)
    

    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
