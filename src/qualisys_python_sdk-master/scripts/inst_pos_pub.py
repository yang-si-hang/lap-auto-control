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


import pandas as pd



time_start = None

inst_right = None
inst_left = None
inst_right_name = 'inst_r'
inst_left_name = 'inst_l'

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
    global inst_left,inst_right
    global inst_left_name, inst_right_name
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
        global inst_left,inst_right
        global inst_left_name, inst_right_name

        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )

        if inst_right_name is not None and inst_right_name in body_index:
            # Extract one specific body
            inst_right_index = body_index[inst_right_name]
            position, rotation = bodies[inst_right_index]
            # print("{} - Pos: {} - Rot: {}".format(inst_right_name, position, rotation))
            # R = np.array(rotation).reshape(3,3)
            # print("R",R)
            # q = matrix2quaternion(R)
            # print("q: ",q)
            # print(f"q: {q.w},{q.x},{q.y},{q.z}")

            inst_right = np.array(position)

        else:
            # Print all bodies
            for position, rotation in bodies:
                print("Pos: {} - Rot: {}".format(position, rotation))

        if inst_left_name is not None and inst_left_name in body_index:
            # Extract one specific body
            inst_left_index = body_index[inst_left_name]
            position1, rotation1 = bodies[inst_left_index]
            # print("{} - Pos: {} - Rot: {}".format(inst_left_name, position, rotation))
            inst_left = np.array(position1)

        else:
            # Print all bodies
            for position, rotation in bodies:
                print("Pos: {} - Rot: {}".format(position, rotation))

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    while not rospy.is_shutdown():
        # Wait asynchronously 5 seconds
        await asyncio.sleep(0.01)
        # left_pub.publish(temp1.astype(np.float32))
        # print('inst_left : ',inst_left)
        # print('inst_right: ',inst_right)
        # print(f'{(time.time()-time_start):.3f} s')




    # Stop streaming
    await connection.stream_frames_stop()


if __name__ == "__main__":
    time_start = time.time()
    rospy.init_node('qualisys_6dof', anonymous=True)
    left_pub = rospy.Publisher('LeftPos', numpy_msg(Floats), queue_size=1)


    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
