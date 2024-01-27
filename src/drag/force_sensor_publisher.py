"""Prints the readings of a Bota Systems Serial sensor.

Usage: python bota_serial_example.py <port>

This example expects a device layout according to
_expected_device_layout, see below.

类中的run为主线程
run中主要进行两件事:
    1、建立线程 _processdata_thread, 不断地读取串口数据并发布话题
    2、在主线程中循环运行 _my_loop, 按键的检测和 _processdata_thread 线程的关闭信号均在此完成，也可以在此添加其他的任务，当检测到按键后，才会跳出该循环，执行run的后续退出任务
"""

import sys
import struct
import time
import threading

from collections import namedtuple

import serial
from crc import Calculator, Configuration
import rospy
from geometry_msgs.msg import WrenchStamped, Wrench
from std_msgs.msg import Header
import signal
import sys

import select
import tty
import termios

class keyboard_monitor_class(object):
    def __init__(self) -> None:
        signal.signal(signal.SIGINT, self.keyboard_interrupt)
        print("Press Ctrl+C to exit...")
        print("键盘监听开始...")
        # 将终端设置为非规范模式,不修改的话需要回车，而且回车也会被读到
        self.orig_settings = termios.tcgetattr(sys.stdin)
        self.termios_resetted_flag = False
        tty.setcbreak(sys.stdin)
        pass

    def keyboard_interrupt(self,signal, frame):
        print("Keyboard Interrupt detected! (class)")
        # sys.exit(0) #用 raise 或者这个皆可
        raise KeyboardInterrupt 

    def detect(self):
        # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
        return select.select([sys.stdin], [], [], 0.1)

    def read_char(self):
        return sys.stdin.read(1)
    
    def monitor_stop(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
        self.termios_resetted_flag = True
        print("------恢复终端设置------")
        print("------键盘监听结束------")
        return 0
    
    def __del__(self):
        if not self.termios_resetted_flag:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
            print("------恢复终端设置------")


class BotaSerialSensor:

    BOTA_PRODUCT_CODE = 123456
    BAUDERATE = 460800
    SINC_LENGTH = 8
    CHOP_ENABLE = 0
    FAST_ENABLE = 0
    FIR_DISABLE = 1
    TEMP_COMPENSATION = 0 # 0: Disabled (recommended), 1: Enabled
    USE_CALIBRATION = 1 # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0 # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4 # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b'\xAA'
    # Note that the time step is set according to the sinc filter size!
    time_step = 0.01

    def __init__(self,port = '/dev/ttyACM0'):
        rospy.init_node('BotaSerialSensor')
        self._port = port
        self._ser = serial.Serial()
        self._pd_thread_stop_event = threading.Event()
        DeviceSet = namedtuple('DeviceSet', 'name product_code config_func')
        self._expected_device_layout = {0: DeviceSet('BFT-SENS-SER-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}
        self._status = None
        self.setted = False
        self._fx = 0.0
        self._fy = 0.0
        self._fz = 0.0
        self._mx = 0.0
        self._my = 0.0
        self._mz = 0.0
        self._timestamp = 0.0
        self._temperature = 0.0

        self._time_time = 0.0
        self._update_time = 0.0
        self.frequency = 0
        self._pub = rospy.Publisher('Bota_force_sensor/wrenchstamped',WrenchStamped, queue_size=1)
        self.wrench_stamped_msg = WrenchStamped()
        self.wrench_msg = Wrench()




    def bota_sensor_setup(self):
        print("Trying to setup the sensor.")

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes('C', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,C,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,C,0', 'ascii'), out):
            print("Failed to go to CONFIG mode.")
            return False

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        #print(comm_setup)
        cmd = bytes(comm_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,c,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,c,0', 'ascii'), out):
            print("Failed to set communication setup.")
            return False
        self.time_step = 0.00001953125*self.SINC_LENGTH
        print("Timestep: {}".format(self.time_step))

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        #print(filter_setup)
        cmd = bytes(filter_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,f,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,f,0', 'ascii'), out):
            print("Failed to set filter setup.")
            return False

        # Go to RUN mode
        cmd = bytes('R', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,R,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,R,0', 'ascii'), out):
            print("Failed to go to RUN mode.")
            return False
        self.setted = True
        return True

    def contains_bytes(self, subsequence, sequence):
        return subsequence in sequence

    def _processdata_thread(self):
        while not self._pd_thread_stop_event.is_set():
            frame_synced = False
            crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
            crc_calculator = Calculator(crc16X25Configuration)

            while not frame_synced and not self._pd_thread_stop_event.is_set():
                possible_header = self._ser.read(1)
                if self.FRAME_HEADER == possible_header:
                    #print(possible_header)
                    data_frame = self._ser.read(34)
                    crc16_ccitt_frame = self._ser.read(2)

                    crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                    checksum = crc_calculator.checksum(data_frame)
                    if checksum == crc16_ccitt:
                        print("Frame synced")
                        frame_synced = True
                    else:
                        self._ser.read(1)

            while frame_synced and not self._pd_thread_stop_event.is_set():            
                start_time = time.perf_counter()
                frame_header = self._ser.read(1)

                if frame_header != self.FRAME_HEADER:
                    print("Lost sync")
                    frame_synced = False
                    break

                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)

                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.checksum(data_frame)
                if checksum != crc16_ccitt:
                    print("CRC mismatch received")
                    break

                self.wrench_stamped_msg.header = Header(stamp=rospy.Time.now())

                self._status = struct.unpack_from('H', data_frame, 0)[0]

                self._fx = struct.unpack_from('f', data_frame, 2)[0]
                self._fy = struct.unpack_from('f', data_frame, 6)[0]
                self._fz = struct.unpack_from('f', data_frame, 10)[0]
                self._mx = struct.unpack_from('f', data_frame, 14)[0]
                self._my = struct.unpack_from('f', data_frame, 18)[0]
                self._mz = struct.unpack_from('f', data_frame, 22)[0]

                self._timestamp = struct.unpack_from('I', data_frame, 26)[0]
                self._temperature = struct.unpack_from('f', data_frame, 30)[0]
                time_diff = time.perf_counter() - start_time

                self.wrench_msg.force.x = self._fx
                self.wrench_msg.force.y = self._fy
                self.wrench_msg.force.z = self._fz

                self.wrench_msg.torque.x = self._mx 
                self.wrench_msg.torque.y = self._my
                self.wrench_msg.torque.z = self._mz

                # Set the Wrench message in the WrenchStamped message
                self.wrench_stamped_msg.wrench = self.wrench_msg

                self._pub.publish(self.wrench_stamped_msg)


                #循环频率监测
                time_step = time.time() - self._time_time
                self.frequency = 1.0/time_step
                self._time_time = time.time()

                #
                
                
                
 

    def _my_loop(self):
        
        keyboard_monitor = keyboard_monitor_class()
        
        try:
            while True:
                # print('Run my loop')
                print('===============================')
                print(f'数据更新频率: {self.frequency}')
                print("Status {}".format(self._status))
                print("Fx {}".format(self._fx))
                print("Fy {}".format(self._fy))
                print("Fz {}".format(self._fz))
                print("Mx {}".format(self._mx))
                print("My {}".format(self._my))
                print("Mz {}".format(self._mz))

                print("Timestamp {}".format(self._timestamp))

                print("Temperature {}\n".format(self._temperature))

                time.sleep(1)

                # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
                rlist, _, _ = keyboard_monitor.detect()
                if rlist:
                    # 读取单个字符并处理
                    input_data = keyboard_monitor.read_char()
                    print("You typed:", input_data)

                else:
                    input_data=''
                    # count=0

            
            
        # 程序中断处理        
        except KeyboardInterrupt:
            self._pd_thread_stop_event.set()
            print("Keyboard Interrupt detected!  (except)")
            print('stopped')
        

        # 恢复终端设置
        finally:
            keyboard_monitor.monitor_stop()
 


    def run(self):

        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10

        try:
            self._ser.open()
            print("Opened serial port {}".format(self._port))
        except:
            raise BotaSerialSensorError('Could not open port')

        if not self._ser.is_open:
            raise BotaSerialSensorError('Could not open port')

        for i in range(10):
            if not self.setted:
                self.bota_sensor_setup()
            else:
                break
        if not self.setted:
            print('Could not setup sensor!')
            return

        #check_thread = threading.Thread(target=self._check_thread)
        #check_thread.start()
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()
        
        device_running = True

        if device_running:
            self._my_loop()

        self._pd_thread_stop_event.set()
        proc_thread.join()
        #check_thread.join()

        self._ser.close()
        print(f'串口已关闭')

        if not device_running:
            raise BotaSerialSensorError('Device is not running')

    @staticmethod
    def _sleep(duration, get_now=time.perf_counter):
        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()

class BotaSerialSensorError(Exception):
    def __init__(self, message):
        super(BotaSerialSensorError, self).__init__(message)
        self.message = message


if __name__ == '__main__':

    print('bota_serial_example started')

   
    try:
        bota_sensor_1 = BotaSerialSensor()
        bota_sensor_1.run()
    except BotaSerialSensorError as expt:
        print('bota_serial_example failed: ' + expt.message)
        sys.exit(1)
