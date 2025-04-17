'''
通过话题持续发布指定的 rigid_calibrate 刚体的位姿

(该程序的设计初衷是用于动捕和机器人之间的标定，本程序负责发布刚体信息，其他程序进行记录和其他操作)
'''

import numpy as np
import time
import time,os

import sys
import rospy
from spatialmath.base import *
import pkg_resources
import xml.etree.ElementTree as ET
import qtm_rt
import asyncio

from geometry_msgs.msg import PoseStamped


# sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set

sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts")
import rokae_basic_fun 
#=======================================================================================================
rospy.init_node('qualisys_rob_CAL2_pub', anonymous=True)

rigid_calibrate = lap_set.rigid_names['qualisys_rob_calibration']
assert rigid_calibrate is not None, f'未正确设置刚体名称'



time_start = None
T_cam_rigid = np.identity(4)
QTM_FILE = pkg_resources.resource_filename("qtm_rt", "data/Demo.qtm")

pub = rospy.Publisher("qualisys_cal_pub", PoseStamped, queue_size=1)
pose_msg = PoseStamped()




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

async def qualisys_read_pub():
    global pub, pose_msg


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


    def on_packet(packet):
        global pub, pose_msg
        
        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )


        #此处只特定刚体 rigid_calibrate
        rigid_index = body_index[rigid_calibrate]
        position, rotation = bodies[rigid_index]
        
        R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
        
        T_cam_rigid[:3, :3] = R
        T_cam_rigid[:3, 3] = np.array(position).squeeze()/1000 #除以1000后单位为米

        if not np.isnan(position[0]):  
            print(f"{rigid_calibrate} \n{R}\n{position}")
            # print(f"{rigid_calibrate} get")

            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = position[0]/1000
            pose_msg.pose.position.y = position[1]/1000
            pose_msg.pose.position.z = position[2]/1000

            q = r2q(R)
            pose_msg.pose.orientation.w = q[0]
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]

            pub.publish(pose_msg)

            pass
        
        # else:
            # print(f'can\'t get rigid message!!!')

        # q = r2q(R)
        # print("q: ",q)

        '''数据字符串 xyz、                                                      wxyz、'''
        # data = f'{position[0]/1000},{position[1]/1000},{position[2]/1000},\t{q[0]},{q[1]},{q[2]},{q[3]}\n'
    
        # if not np.isnan(position[0]): print(f'{data}')
        # print(f'{(time.perf_counter()-time_start):.6f} s')
        # print("{} - Pos: {} - Rot: {}".format(calibration_rigid_calibrate, position, rotation))
        

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    # while not rospy.is_shutdown():
    # while True:
    #     # Wait asynchronously seconds
    #     await asyncio.sleep(1)
    #     pass
    
    while not rospy.is_shutdown():
        # while True:
        # Wait asynchronously seconds
        await asyncio.sleep(1)
        pass

    # try:
    #     while True:
    #         await asyncio.sleep(1) #此异步期间会进行on_packet，更新 quealisys 数据
    
    # except:
    #     # Stop streaming
    #     await connection.stream_frames_stop()





if __name__ == "__main__":
    time_start = time.perf_counter()

    try:
        # Run our asynchronous function until complete
        asyncio.get_event_loop().run_until_complete(qualisys_read_pub())
    except KeyboardInterrupt:
        rospy.signal_shutdown()
   


    
