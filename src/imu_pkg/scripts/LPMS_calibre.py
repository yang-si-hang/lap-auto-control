#用于获取陀螺仪加速度偏置量
import  time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import urx
import math3d as m3d
# import keyboard



sys.path.append("/home/wyh/IMU_drivers/openzen/build")
import openzen



import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5 import QtWidgets

sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set


g_std = 9.78
data_frequency=200

imu_data = []
quat_data = []
acc_data = []

#0：对应轴竖直朝上时的加速度度数  1：朝下
acc_x0 = 0
acc_x1 = 0
acc_y0 = 0
acc_y1 = 0
acc_z0 = 0
acc_z1 = 0

if __name__ == '__main__':

    rob = urx.Robot(lap_set.robot_ip)
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(2, (0, 0, 0.1))
    trans = rob.get_pose()


    openzen.set_log_level(openzen.ZenLogLevel.Warning)

    error, client = openzen.make_client()
    if not error == openzen.ZenError.NoError:
        print ("Error while initializing OpenZen library")
        sys.exit(1)

    error = client.list_sensors_async()

    error, sensor = client.obtain_sensor_by_name("Bluetooth", "00:04:3E:6C:52:83", 115200)

    if not error == openzen.ZenSensorInitError.NoError:
        print ("Error connecting to sensor")
        sys.exit(1)

    print ("Connected to sensor !")

    imu = sensor.get_any_component_of_type(openzen.component_type_imu)
    if imu is None:
        print ("No IMU found")
        sys.exit(1)

    ## read bool property
    error, is_streaming = imu.get_bool_property(openzen.ZenImuProperty.StreamData)
    if not error == openzen.ZenError.NoError:
        print ("Can't load streaming settings")
        sys.exit(1)

    print ("Sensor is streaming data: {}".format(is_streaming))

    print("\n>> Set and get IMU settings")
    # test to print imu ID
    error = imu.set_int32_property(openzen.ZenImuProperty.Id, 0)
    error, imu_id = imu.get_int32_property(openzen.ZenImuProperty.Id)
    print("IMU ID: {}".format(imu_id))

    # test to set freq
    error = imu.set_int32_property(openzen.ZenImuProperty.SamplingRate, data_frequency)
    error, freq = imu.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate: {}".format(freq))


    time.sleep(1)
    print()
    print("wait")
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
            if(imu_data[-1].timestamp - imu_data[0].timestamp) > 5:
                break




    #----------------- x0 ---------------------------------------------------------------

    print("x0")

    imu_data = []
    acc_data = []
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if 6.98<imu_data[-1].timestamp - imu_data[0].timestamp <7.03:
            print("will move after 3s...")

        if imu_data[-1].timestamp - imu_data[0].timestamp > 10:
            break

    trans = rob.get_pose()
    trans.orient = np.array([   [0, 0,  1],
                            [0, -1,  0],
                            [1, 0,  0]])
    
    rob.set_pose(trans, acc=0.5, vel=0.2)
    imu_data = []
    acc_data = []
    while len(acc_data) < 2000:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if imu_data[-1].timestamp - imu_data[0].timestamp < 3:
            continue

        acc_data.append(imu_data[-1].a[0]*g_std)
        print(len(acc_data))

    
    acc_x0 = np.mean(acc_data)
    print("acc_x0: ", acc_x0,"\n",acc_data[0],' ',acc_data[-1])
    print(len(acc_data))

    #----------------- x1 ---------------------------------------------------------------
    print("x1")

    imu_data = []
    acc_data = []
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        if 6.98<imu_data[-1].timestamp - imu_data[0].timestamp <7.03:
            print("will move after 3s...")
        if imu_data[-1].timestamp - imu_data[0].timestamp > 10:
            break

    trans = rob.get_pose()
    trans.orient = np.array([   [0, 0,  1],
                            [0, 1,  0],
                            [-1, 0,  0]])
    
    rob.set_pose(trans, acc=0.5, vel=0.2)
    imu_data = []
    acc_data = []
    
    while len(acc_data) < 2000:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if imu_data[-1].timestamp - imu_data[0].timestamp < 3:
            continue

        acc_data.append(imu_data[-1].a[0]*g_std)
        print(len(acc_data))

    
    acc_x1 = np.mean(acc_data)
    print("acc_x1: ", acc_x1,"\n",acc_data[0],' ',acc_data[-1])


#----------------- y0 ---------------------------------------------------------------
    print("y0")


    imu_data = []
    acc_data = []
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        if 6.98<imu_data[-1].timestamp - imu_data[0].timestamp <7.03:
            print("will move after 3s...")
        if imu_data[-1].timestamp - imu_data[0].timestamp > 10:
            break


    trans = rob.get_pose()
    trans.orient = np.array([   [0,     0,      1],
                                [1,     0,      0],
                                [0,     1,      0]])

    rob.set_pose(trans, acc=0.5, vel=0.2)
    imu_data = []
    acc_data = []
    while len(acc_data) < 2000:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if imu_data[-1].timestamp - imu_data[0].timestamp < 3:
            continue

        acc_data.append(imu_data[-1].a[1]*g_std)
        print(len(acc_data))

    
    acc_y0 = np.mean(acc_data)
    print("acc_y0: ", acc_y0,"\n",acc_data[0],' ',acc_data[-1])


    #----------------- y1 ---------------------------------------------------------------
    print("y1")

    imu_data = []
    acc_data = []
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        if 6.98<imu_data[-1].timestamp - imu_data[0].timestamp <7.03:
            print("will move after 3s...")
        if imu_data[-1].timestamp - imu_data[0].timestamp > 10:
            break


    trans = rob.get_pose()
    trans.orient = np.array([   [0,     0,      1],
                                [-1,     0,      0],
                                [0,     -1,      0]])
    
    rob.set_pose(trans, acc=0.5, vel=0.2)
    imu_data = []
    acc_data = []
    while len(acc_data) < 2000:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if imu_data[-1].timestamp - imu_data[0].timestamp < 3:
            continue

        acc_data.append(imu_data[-1].a[1]*g_std)
        print(len(acc_data))
    
    acc_y1 = np.mean(acc_data)
    print("acc_y1: ", acc_y1,"\n",acc_data[0],' ',acc_data[-1])



    #----------------- z0 ---------------------------------------------------------------

    print("z0")

    imu_data = []
    acc_data = []
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        if 6.98<imu_data[-1].timestamp - imu_data[0].timestamp <7.03:
            print("will move after 3s...")
        if imu_data[-1].timestamp - imu_data[0].timestamp > 10:
            break



    trans = rob.get_pose()
    trans.orient = np.array([   [1,     0,      0],
                                [0,     1,      0],
                                [0,     0,      1]])

    rob.set_pose(trans, acc=0.5, vel=0.2)
    imu_data = []
    acc_data = []
    while len(acc_data) < 2000:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if imu_data[-1].timestamp - imu_data[0].timestamp < 3:
            continue

        acc_data.append(imu_data[-1].a[2]*g_std)
        print(len(acc_data))

    
    acc_z0 = np.mean(acc_data)
    print("acc_z0: ", acc_z0,"\n",acc_data[0],' ',acc_data[-1])


    #----------------- z1 ---------------------------------------------------------------
    print("z1")

    imu_data = []
    acc_data = []
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        if 6.98<imu_data[-1].timestamp - imu_data[0].timestamp <7.03:
            print("will move after 3s...")
        if imu_data[-1].timestamp - imu_data[0].timestamp > 10:
            break



    trans = rob.get_pose()
    trans.orient = np.array([   [1,     0,      0],
                                [0,     -1,      0],
                                [0,     0,      -1]])

    rob.set_pose(trans, acc=0.5, vel=0.2)
    imu_data = []
    acc_data = []
    while len(acc_data) < 2000:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data.append( zenEvent.data.imu_data)
        
        if imu_data[-1].timestamp - imu_data[0].timestamp < 3:
            continue

        acc_data.append(imu_data[-1].a[2]*g_std)
        print(len(acc_data))

    
    acc_z1 = np.mean(acc_data)
    print("acc_z1: ", acc_z1,"\n",acc_data[0],' ',acc_data[-1])


    delta = [(acc_x0+acc_x1)/2 , (acc_y0+acc_y1)/2, (acc_z0+acc_z1)/2]
    print("delta: ",delta)

    print ("Streaming of sensor data complete")
    sensor.release()
    client.close()
    print("OpenZen library was closed")
    sys.exit(1)


        
    




        

    
