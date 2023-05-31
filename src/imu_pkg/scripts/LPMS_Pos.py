###########################################################################
#
# OpenZen Python example
#
# Make sure the openzen.pyd (for Windows) or openzen.so (Linux/Mac, simply rename 
# libOpenZen.so to openzen.so) are in the same folder as this file.
#
# If you want to connect to USB sensors on Windows, the file SiUSBXp.dll
# should also be in the same folder.
#
# Python interfaces definitions could be find in `../src/bindings/OpenZenPython.cpp`.
# Function, enum and property names of the OpenZen Python interface are in double quotes.
#
###########################################################################


#
# Python examples on connecting to sensor, toggling sensor settings and print sensor data.
#
# Check our docs for more https://lpresearch.bitbucket.io/openzen/latest/getting_started.html
#

import  time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import urx
# import keyboard



sys.path.append("/home/wyh/IMU_drivers/openzen/build")
import openzen



import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5 import QtWidgets


import signal

# define a signal handler function
def signal_handler(sig, frame):
    # print('Stopping infinite loop...')
    raise KeyboardInterrupt

# set up the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)


acc_bias = np.array( [0.012536895323395747, 0.01717755648046726, 0.046137140538990984] )
acc_bias = np.array( [0.009934463513195269, 0.027290225948095603, 0.04809187778860391] )
acc_bias = np.array( [0.01064629765570313, 0.021558780201674566, 0.06197530283689545] )



g0_init = 0  #init 坐标系下的，会在init时转换完成
g0_init = 0
g0_data = []
imu_g0_data = []

a0_drift_rate = 0
delta_a = []

acc_drift = 0
acc_drift_rate = 0
g0_threshold = 0.02

q0_data = []
q0 = np.array([0, 0, 0, 1])

time_drift_start = []


imu_read = []
acc_data = []
acc_data_init = []
acc_data_init1 = []
acc_data_init2 = []

data_frequency = 200
g_std = 9.78

R_0_init = np.identity(3)  #角标先上后下，此为，在0坐标系，及开机坐标系下，看运行init函数时的坐标系
R_init_0 = np.identity(3)
R_0_i = np.identity(3)
R_init_i = np.identity(3)
acc_temp = np.array([0,0,0])

a_threshold = 0.05
w_threshold = 0.2

static_flag = 1

v_init = []
x_init = []


# Create application and main window
app = QtWidgets.QApplication([])
win_acc = pg.GraphicsLayoutWidget()
win_v = pg.GraphicsLayoutWidget()

win_acc.setWindowTitle("acc")
win_v.setWindowTitle("v")

plot_acc = win_acc.addPlot()
plot_acc_data_x = plot_acc.plot(np.random.normal(size=5*200), pen='r')
plot_acc_data_y = plot_acc.plot(np.random.normal(size=5*200), pen='g')
plot_acc_data_z = plot_acc.plot(np.random.normal(size=5*200), pen='b')
plot_acc.setXRange(0, 5*200)
plot_acc.setYRange(-0.4,0.4)
win_acc.addItem(plot_acc)
win_acc.show()

plot_v = win_v.addPlot()
plot_v_data_x = plot_v.plot(np.random.normal(size=5*200),pen='r')
plot_v_data_y = plot_v.plot(np.random.normal(size=5*200),pen='g')
plot_v_data_z = plot_v.plot(np.random.normal(size=5*200),pen='b')
plot_v.setXRange(0, 5*200)
plot_v.setYRange(-20,20)
win_v.addItem(plot_v)
win_v.show()


# Start the event loop
# app.exec_()

def acc_init():
    global g0_init
    global g0_data
    global imu_g0_data
    global a0_drift_rate
    global g0_threshold
    global q0_data 
    global q0
    global time_drift_start
    global R_0_init 
    global R_init_0
    global acc_drift_rate
    global acc_bias


    i_start = 0
    i_end = 0

    mean_num = 50   #先对开头和结尾的一些向量进行平均，以此为依据计算 delta a ，将其和a_threshold比较
    a0_start = np.mean(g0_data[0:mean_num],axis=0)
    a0_end = np.mean(g0_data[(len(g0_data)-mean_num) : len(g0_data)],axis=0)
    print("a0_start",a0_start)
    print("a0_end",a0_end)
    q0 = np.mean(q0_data,axis=0)
    q0 = q0/np.linalg.norm(q0)
    q0 = np.concatenate((q0[1:],[q0[0]]))
    R_0_init = R.from_quat(q0).as_matrix()#from 的q顺序为：xyzw
    R_init_0 = R_0_init.T
    print("q0: ",q0,"  norm: ", np.linalg.norm(q0) )
    print ("R_0_init: \n",R_0_init)
    print ("R_init_0: \n",R_init_0)


    for i in range(mean_num,len(g0_data)):
        if np.linalg.norm( g0_data[i] - a0_start ) > g0_threshold:
            i_start = i
            time_drift_start.append(imu_g0_data[i_start-1].timestamp)
            # print(g0_data[i_start] - a0_start," norm: ",np.linalg.norm( g0_data[i_start] - a0_start )," > ",g0_threshold)
            print("i_start: ",i_start," / ",len(g0_data))
            break
        else:
            pass

    for i in range(mean_num+1,len(g0_data)+1):
        if np.linalg.norm( (g0_data[-i]) - a0_end ) > g0_threshold:
            i_end = len(g0_data)-i
            # print(g0_data[i_end]-a0_end," norm: ",np.linalg.norm( (g0_data[i_end]) - a0_end )," > ",g0_threshold)
            print("i_end: ",i_end," / ",len(g0_data))
            break
        else:
            pass

    a0_drift = np.mean(g0_data[i_end+1:],axis=0)-np.mean(g0_data[0:i_start],axis=0)
    if i_end > i_start:
        a0_drift_rate = a0_drift / (imu_g0_data[i_end+1].timestamp - imu_g0_data[i_start-1].timestamp)
    else:
        a0_drift_rate = 0
    
    acc_drift_rate = a0_drift_rate
    print("a0_drift: ",a0_drift)
    print("a0_drift_rate: ",a0_drift_rate)

    # for i in range(i_start, len(g0_data)):
    #     g0_data[i] = g0_data[i] - a0_drift_rate * (imu_g0_data[i].timestamp - imu_g0_data[i_start-1].timestamp)
    
    g0_init = np.mean(g0_data, axis=0)

    print("g0_init: ",g0_init)
    print("g0_init norm: ",np.linalg.norm(g0_init))


    return


    

    










if __name__ == '__main__':
    # rospy.init_node('LPMS_Pos', anonymous=True)


    rob = urx.Robot("192.168.100.102")
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(2, (0, 0, 0.1))
    trans = rob.get_pose()

   
    trans = rob.get_pose()
    trans.orient = np.array([   [0, 0,  1],
                            [0, -1,  0],
                            [1, 0,  0]])
    
    rob.set_pose(trans, acc=0.5, vel=0.2)
   
    openzen.set_log_level(openzen.ZenLogLevel.Warning)

    error, client = openzen.make_client()
    if not error == openzen.ZenError.NoError:
        print ("Error while initializing OpenZen library")
        sys.exit(1)

    error = client.list_sensors_async()

    # # check for events
    # sensor_desc_connect = None
    # while True:
    #     zenEvent = client.wait_for_next_event()
    #
    #     if zenEvent.event_type == openzen.ZenEventType.SensorFound:
    #         print ("Found sensor {} on IoType {}".format( zenEvent.data.sensor_found.name,
    #             zenEvent.data.sensor_found.io_type))
    #         if sensor_desc_connect is None:
    #             sensor_desc_connect = zenEvent.data.sensor_found
    #
    #     if zenEvent.event_type == openzen.ZenEventType.SensorListingProgress:
    #         lst_data = zenEvent.data.sensor_listing_progress
    #         print ("Sensor listing progress: {} %".format(lst_data.progress * 100))
    #         if lst_data.complete > 0:
    #             break
    # print ("Sensor Listing complete")
    #
    # if sensor_desc_connect is None:
    #     print("No sensors found")
    #     sys.exit(1)

    # connect to the first sensor found, more on https://lpresearch.bitbucket.io/openzen/latest/io_systems.html
    # error, sensor = client.obtain_sensor(sensor_desc_connect)

    # or connect to a sensor by name
    # error, sensor = client.obtain_sensor_by_name("SiUsb", "ig1pcan000028", 921600)

    # or connect to a sensor by COM
    # error, sensor = client.obtain_sensor_by_name("WindowsDevice", "//./COM25", 921600)

    # or connect to a Bluetooth sensor (LPMS-B2)
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


    print()
    print("wait")



    ## load the alignment matrix from the sensor
    ## some sensors don't support this (for example IG1, BE1)
    #error, accAlignment = imu.get_array_property_float(openzen.ZenImuProperty.AccAlignment)
    #if not error == openzen.ZenError.NoError:
    #    print ("Can't load alignment")
    #    sys.exit(1)

    #if not len(accAlignment) == 9:
    #    print ("Loaded Alignment has incosistent size")
    #    sys.exit(1)

    #print ("Alignment loaded: {}".format(accAlignment))

    ## store float array
    #error = imu.set_array_property_float(openzen.ZenImuProperty.AccAlignment, accAlignment)

    #if not error == openzen.ZenError.NoError:
    #    print ("Can't store alignment")
    #    sys.exit(1)

    #print("Stored alignment {} to sensor".format(accAlignment))

    # start streaming data
    
    
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_g0_data.append( zenEvent.data.imu_data)
            if(imu_g0_data[-1].timestamp - imu_g0_data[0].timestamp) > 5:
                break


    runSome = 0
    imu_g0_data = []
    imu_data_temp = None
    last_data = None
    while True:
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:


            if imu_data_temp is None:
                imu_data_temp = zenEvent.data.imu_data
                last_data = imu_data_temp
            else:
                last_data = imu_data_temp
                imu_data_temp = zenEvent.data.imu_data

            

            g0_data.append(np.array(imu_data_temp.a)*g_std - acc_bias)
            imu_g0_data.append(imu_data_temp)
            q0_data.append(imu_data_temp.q)
            
            delta_time = imu_data_temp.timestamp - last_data.timestamp
            delta_a.append((np.array(imu_data_temp.a) - np.array(last_data.a))*g_std)

            print("runSome : {}".format(runSome))
            # print ("ts: {:f} s".format(imu_data_temp.timestamp))
            # print("delta time: {}".format(delta_time))
            # print("delta a: {}".format(delta_a[runSome]))
            # print("|delta a|: {}".format(np.linalg.norm(delta_a[runSome])))
            # if delta_time > 0.006:
            #     print("----------------------------------------------------------------------------------")
            #     print("----------------------------------------------------------------------------------")
            #     print("----------------------------------------------------------------------------------")
            #     print("----------------------------------------------------------------------------------")
            #     print("----------------------------------------------------------------------------------")
            # print ("A: {} g".format(imu_data_temp.a))
            print ("A norm: {} ".format(np.linalg.norm(imu_data_temp.a)*g_std))
            # print ("G1: {} degree/s".format(imu_data_temp.g1))   # depending on sensor, gyro data is outputted to g1, g2 or both
            print ("G1 norm: {} degree/s".format(np.linalg.norm(imu_data_temp.g1)))   # depending on sensor, gyro data is outputted to g1, g2 or both

            # print ("G2: {} degree/s".format(imu_data_temp.g2))   # read more on https://lpresearch.bitbucket.io/openzen/latest/getting_started.html#id1
            # print ("B: {} microT".format(imu_data_temp.b))
            # print ("q: {} ".format(imu_data_temp.q))
            runSome = runSome + 1

        # if runSome ==5:
        #     time.sleep(3)
        #     print("sleep...................")

        if runSome > 10*200-1:
            break

    print("delta a mean: {}".format(sum(delta_a)/len(delta_a)))
    print("delta a max: {}".format(max(np.linalg.norm(delta_a,axis=1))))
    print("variance of acc: ",np.array(g0_data).var(axis=0))

    acc_init()
    
    print_flag = 0
    interval = 20
    i = 0
    try:
        while  True:
        # while  not rospy.is_shutdown():
            

            zenEvent = client.wait_for_next_event()
            time_start = time.time()
            imu_data_temp = zenEvent.data.imu_data
            imu_read.append( imu_data_temp )
            
            q = np.array(imu_data_temp.q)
            q = np.concatenate((q[1:],[q[0]]))


            R_0_i = R.from_quat(q).as_matrix()
            R_init_i = R_init_0 @ R_0_i
            
            


            # acc_temp = np.array(imu_data_temp.a)*g_std-acc_drift_rate * (imu_data_temp.timestamp - time_drift_start[-1])
            acc_temp = np.array(imu_data_temp.a)*g_std - acc_bias
            acc_data.append(acc_temp)
            acc_temp = (R_init_i @ acc_temp.T).T
            acc_temp = acc_temp- g0_init

            if len(imu_read) < 2:
                v_init.append(np.array([0, 0, 0]))
                x_init.append(np.array([0, 0, 0]))
                acc_data_init.append(np.array([0, 0, 0]))
                print("a_init: ",np.array([0, 0, 0]))
                continue

            # 检测是否静止，计算速度、加速度---------------------------------------------------------------------------------
            delta_time = imu_read[-1].timestamp - imu_read[-2].timestamp
            # if np.linalg.norm(acc_temp) < a_threshold and np.linalg.norm(imu_data_temp.g1) < w_threshold:
            if np.linalg.norm(acc_temp) < a_threshold and np.linalg.norm(imu_data_temp.g1) < 0:
                # print("static  a:",np.linalg.norm(acc_temp),"\n        w:",np.linalg.norm(imu_data_temp.g1))
                acc_temp = np.array([0, 0, 0])
                v_init.append(np.array([0, 0, 0]))
                static_flag = 1

            else:
                v_temp = delta_time*acc_temp + v_init[-1]
                v_init.append( v_temp )
                static_flag = 0
            # ------------------------------------------------------------------------------------------------------------

            acc_data_init.append(acc_temp)

            x_init.append(x_init[-1] +  v_init[-1]*delta_time + 0.5*(delta_time**2)*acc_data_init[-1] )

            if len(imu_read)%interval == 0:
                # print("------------------------------------------------------------")
                # # print("quat: ",q)
                # # print("R_init_i:\n",R_init_i)
                # print("a_init: ",acc_temp)
                # # print("w : ",imu_data_temp.g1)
                # print("v : ",v_init[-1])
                # # print("x : ",x_init[-1])
                # print("plot")


                # plot_acc_data_x.setData([sub_list[0] for sub_list in acc_data_init[:]])
                # plot_acc_data_y.setData([sub_list[1] for sub_list in acc_data_init[:]])
                # plot_acc_data_z.setData([sub_list[2] for sub_list in acc_data_init[:]])

                # plot_v_data_x.setData([sub_list[0] for sub_list in v_init[:]])
                # plot_v_data_y.setData([sub_list[1] for sub_list in v_init[:]])
                # plot_v_data_z.setData([sub_list[2] for sub_list in v_init[:]])

                # plot_acc.setXRange(0, len(acc_data_init))
                # plot_v.setXRange(0, len(acc_data_init))
                if len(imu_read) > 5 * 200:
                    
                    plot_acc_data_x.setData([sub_list[0] for sub_list in acc_data_init[-5*200:]])
                    plot_acc_data_y.setData([sub_list[1] for sub_list in acc_data_init[-5*200:]])
                    plot_acc_data_z.setData([sub_list[2] for sub_list in acc_data_init[-5*200:]])

                    plot_v_data_x.setData([sub_list[0] for sub_list in v_init[-5*200:]])
                    plot_v_data_y.setData([sub_list[1] for sub_list in v_init[-5*200:]])
                    plot_v_data_z.setData([sub_list[2] for sub_list in v_init[-5*200:]])
                    
                    

                else:
                    plot_acc_data_x.setData([sub_list[0] for sub_list in acc_data_init[:]])
                    plot_acc_data_y.setData([sub_list[1] for sub_list in acc_data_init[:]])
                    plot_acc_data_z.setData([sub_list[2] for sub_list in acc_data_init[:]])

                    plot_v_data_x.setData([sub_list[0] for sub_list in v_init[:]])
                    plot_v_data_y.setData([sub_list[1] for sub_list in v_init[:]])
                    plot_v_data_z.setData([sub_list[2] for sub_list in v_init[:]])

                if len(imu_read) == 10 * 200:
                    print("mean1: ", np.mean(acc_data_init[:], axis=0))
                    trans = rob.get_pose()
                    trans.orient.rotate_xb(+np.pi/6)

                    rob.set_pose(trans, acc=0.5, vel=0.2)

                if len(imu_read) == 20 * 200:
                    trans = rob.get_pose()
                    trans.orient = np.array([   [0, 0,  1],
                                            [0, -1,  0],
                                            [1, 0,  0]])
                    
                    rob.set_pose(trans, acc=0.5, vel=0.2)

                if len(imu_read) == 35 * 200:
                    print("mean2: ", np.mean(acc_data_init[-5 * 200:], axis=0))

   

                QtWidgets.QApplication.processEvents()


                # print("calculate once cost: ",(time.time()- time_start)*1000," ms")
                


           
            # acc_temp = np.array(imu_data_temp.a)*g_std
            # acc_data.append(acc_temp)
            # acc_temp = R_init_i @ acc_temp
            # acc_temp = acc_temp- g0_init
            # acc_data_init.append(acc_temp)
            # print("a_init_no_drift_correct: ",acc_temp)


            # acc_temp = np.array(imu_data_temp.a)*g_std
            # acc_data.append(acc_temp)
            # acc_temp = R_init_i @ acc_temp
            # acc_temp = acc_temp- g0_init - acc_drift_rate * (imu_data_temp.timestamp - time_drift_start[-1])
            # acc_data_init.append(acc_temp)
            # print("a_drift_correct_in_init: ",acc_temp)


            # if keyboard.is_pressed('q'):
            #     print("Stopping infinite loop...")


            
            #     break
            # i=i+1
            # if(i>3):
            #     break
            pass

    except KeyboardInterrupt:
        # handle the KeyboardInterrupt exception
        print('Stopping infinite loop...')

    

    print ("Streaming of sensor data complete")
    sensor.release()
    client.close()
    print("OpenZen library was closed")
    sys.exit(1)
