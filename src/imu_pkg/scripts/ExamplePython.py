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
sys.path.append("/home/wyh/IMU_drivers/openzen/build")
import openzen


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
error = imu.set_int32_property(openzen.ZenImuProperty.SamplingRate, 200)
error, freq = imu.get_int32_property(openzen.ZenImuProperty.SamplingRate)
print("Sampling rate: {}".format(freq))


time.sleep(1)
print()



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
runSome = 0
imu_data = None
last_data = None
while True:
    zenEvent = client.wait_for_next_event()

    # check if its an IMU sample event and if it
    # comes from our IMU and sensor component
    if zenEvent.event_type == openzen.ZenEventType.ImuData and \
        zenEvent.sensor == imu.sensor and \
        zenEvent.component.handle == imu.component.handle:


        if imu_data is None:
            imu_data = zenEvent.data.imu_data
            last_data = imu_data
        else:
            last_data = imu_data
            imu_data = zenEvent.data.imu_data
        delta_time = imu_data.timestamp - last_data.timestamp
        print("runSome : {}".format(runSome))
        print ("ts: {:f} s".format(imu_data.timestamp))
        print("delta time: {}".format(delta_time))
        if delta_time > 0.006:
            print("----------------------------------------------------------------------------------")
            print("----------------------------------------------------------------------------------")
            print("----------------------------------------------------------------------------------")
            print("----------------------------------------------------------------------------------")
            print("----------------------------------------------------------------------------------")
        # print ("A: {} g".format(imu_data.a))
        # print ("G1: {} degree/s".format(imu_data.g1))   # depending on sensor, gyro data is outputted to g1, g2 or both
        # print ("G2: {} degree/s".format(imu_data.g2))   # read more on https://lpresearch.bitbucket.io/openzen/latest/getting_started.html#id1
        # print ("B: {} microT".format(imu_data.b))
        # print ("q: {} ".format(imu_data.q))
        runSome = runSome + 1

    # if runSome ==5:
    #     time.sleep(3)
    #     print("sleep...................")

    if runSome > 1000:
        break

print ("Streaming of sensor data complete")
sensor.release()
client.close()
print("OpenZen library was closed")
sys.exit(1)
