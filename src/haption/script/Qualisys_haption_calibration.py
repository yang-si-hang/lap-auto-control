'''
用于眼和机械臂分别固连在h世界的情况，注意眼不在手上。
机械臂末端固连观测目标，B
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
from scipy.linalg import logm
from geometry_msgs.msg import PoseStamped


sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts/my_tools")
import key_signal,eye_hand_calibrate

haption_pose = PoseStamped()
def haption_pose_callback(msg):
    haption_pose = msg


rigid_calibrate = 'calibrate'



time_start = None

fold_path = f'{os.path.dirname(__file__)}/../data/Qualisys_Caibrtion/'
T_0_rob_path = f'{fold_path}T_0_rob.txt'
T_cam_rigid_path = f'{fold_path}T_cam_rigid.txt'
T_0_qualisys_path = f'{fold_path}T_0_cam.txt'

# 使用np进行文件保存时，不需要手动打开文件和清空内容

T_cam_rigid = np.identity(4)





T_0_rob_list = []
T_cam_rigid_list = []




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

async def move_and_measure():
    global T_0_rob_list, T_cam_rigid_list
    keyboard_monitor = key_signal.keyboard_monitor_class()

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


        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )


        rigid_index = body_index[rigid_calibrate]
        position, rotation = bodies[rigid_index]
        
        R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
        
        T_cam_rigid[:3, :3] = R
        T_cam_rigid[:3, 3] = np.array(position).squeeze()

        if not np.isnan(position[0]):  
            # print(f"{rigid_calibrate} \n{R}\n{position}")
            # print(f"{rigid_calibrate} get")
            pass
        
        else:
            print(f'can\'t get rigid message!!!')


        

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    # while not rospy.is_shutdown():
    # while True:
    #     # Wait asynchronously seconds
    #     await asyncio.sleep(1)
    #     pass

    

    try:
        print(f'------- wait for input -------')
        while True:
            # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
            rlist, _, _ = keyboard_monitor.detect()
            if rlist:
                # 读取单个字符并处理
                input_data = keyboard_monitor.read_char()
                print("You typed:", input_data)
                if input_data == '\n':
                    
                    await asyncio.sleep(1)
                    R = q2r([haption_pose.pose.orientation.w,haption_pose.pose.orientation.x,haption_pose.pose.orientation.y,haption_pose.pose.orientation.z])
                    T = np.eye(4)
                    T[:3,:3] = R
                    T[:3,3] = np.array([haption_pose.pose.position.x,haption_pose.pose.position.y,haption_pose.pose.position.z]).reshape(3,1)
                    T_0_rob_list.append(T)
                    T_cam_rigid_list.append(T_cam_rigid)


     # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
       

    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        T_0_rob_catted = np.concatenate(T_0_rob_list,axis=1)
        T_cam_rigid_catted = np.concatenate(T_cam_rigid_catted,axis=1)
        np.savetxt(T_0_rob_path, T_0_rob_catted, delimiter=',')
        np.savetxt(T_cam_rigid_path, T_cam_rigid_catted, delimiter=',')
        print(f'shape of T_0_rob_catted: {T_0_rob_catted.shape}')
        print(f'shape of T_cam_rigid_catted: {T_cam_rigid_catted.shape}')
        T_0_qualisys = eye_hand_calibrate.calibrate_calculate(T_0_rob_catted, T_cam_rigid_catted, save_path= T_0_qualisys_path)
        print(f'T_0_qualisys:\n{T_0_qualisys}')






    

    # Stop streaming
    await connection.stream_frames_stop()




if __name__ == "__main__":

    rospy.init_node('qualisys_haption_calibration', anonymous=True)
    rospy.Subscriber("/haption2/state/pose", PoseStamped, haption_pose_callback)
    time.sleep(1)

    

    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(move_and_measure())
    




    
