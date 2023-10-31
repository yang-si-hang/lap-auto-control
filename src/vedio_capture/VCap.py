import ctypes,multiprocessing,time,os
import cv2
import mss
import numpy as np
import pyaudio, wave


class VCap():
    def __init__(self,) -> None:
        self.index = None
        self.area = None
        self.width = None
        self.height = None
        self.fps = None
        self.frameQueue = multiprocessing.Queue()
        self.audioState = multiprocessing.Queue(1)
        self.exitKey = 27
        self.pauseKey = ord("p")
        self.textPara = {
             "org":(10,30), 
             "fontFace":cv2.FONT_HERSHEY_COMPLEX, 
             "fontScale":1.0, 
             "color":(255, 0, 0),
             "thickness":2
        }

    # 摄像头参数
    def set_cam(self, index=0, width=640, height=480,  fps=30):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps

    # 录屏参数
    def set_scr(self, index=0, fps=50):
        capture = mss.mss()
        self.area = capture.monitors[index]
        self.width, self.height = self.area["width"],self.area["height"]
        self.fps = fps

    def generate_capture(self):
        # 捕获摄像头
        if self.index != None:
            capture = cv2.VideoCapture(self.index)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            return capture
        # 捕获屏幕
        elif self.area != None:
            return mss.mss()

    # 显示窗口参数
    def set_window(self, index=1, topmost=False, fullscreen=True, title="Show"):
        self.showIdx = index
        self.win = title
        self.topmost = topmost
        self.fullscreen = fullscreen
        
    def generate_window(self):
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        if self.topmost:
            cv2.setWindowProperty(self.win, cv2.WND_PROP_TOPMOST, 1)
        area = mss.mss().monitors[self.showIdx]
        cv2.moveWindow(self.win, area["left"], area["top"])
        if self.fullscreen:
            cv2.setWindowProperty(self.win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            print("全屏显示于第%d屏" %self.showIdx)
        else:
            print("窗口显示于第%d屏" %self.showIdx)
    
    # 采集摄像头
    def grab_cam(self):
        capture = self.generate_capture()
        self.generate_window()
        key = None
        pause = False

        ret, frame = capture.read()
        if ret:
            self.frameQueue.put(frame.shape[:2])
        else:
            return False

        while (key != self.exitKey):
            if key == self.pauseKey:
                pause = not pause
            
            ret, frame = capture.read()
            timestamp = int(capture.get(cv2.CAP_PROP_POS_MSEC))
            if not ret:
                text = "连接已断开，按任意键结束"
                print(text)
                frame = cv2.putText(frame, text, **self.textPara)
                cv2.imshow(self.win, frame)
                key = cv2.waitKey(100)&0xFF
                break 

            if pause:
                frame = cv2.putText(frame, "pause", **self.textPara)
                cv2.imshow(self.win, frame)
            else:
                cv2.imshow(self.win, frame)
                self.frameQueue.put((timestamp, frame), block=False)
            # 接收键盘输入
            key = cv2.waitKey(1)&0xFF

        self.audioState.put(None)
        self.frameQueue.put(None)
        capture.release()
        cv2.destroyAllWindows()
    
    # 采集屏幕
    def grab_scr(self):
        get_timestamp = ctypes.windll.kernel32.GetTickCount64
        capture = self.generate_capture()
        self.generate_window()
        key = None
        pause = False

        frame = capture.grab(self.area)
        self.frameQueue.put(np.array(frame).shape[:2])

        while (key != self.exitKey):
            if key == self.pauseKey:
                pause = not pause

            frame = capture.grab(self.area)
            timestamp = int(get_timestamp())
            frame = np.array(frame)

            if pause:
                frame = cv2.putText(frame, "pause", **self.textPara)
                cv2.imshow(self.win, frame)
            else:
                cv2.imshow(self.win, frame)
                self.frameQueue.put((timestamp, frame), block=False)

            # 接收键盘输入
            key = cv2.waitKey(1)&0xFF

        self.audioState.put(None)
        self.frameQueue.put(None)
        cv2.destroyAllWindows()

    # 写入视频文件
    def save(self, fileName, enc, suffix):
        fourcc = cv2.VideoWriter_fourcc(*enc)
        height,width = self.frameQueue.get()

        videoN = fileName + '.' + suffix
        timeN = videoN + ".txt"
        videoF = cv2.VideoWriter(videoN, fourcc, self.fps, (width, height))
        timeF = open(timeN, "wb", buffering=0)

        print("开始录制 @ %d * %d * %dHz" %(width, height, self.fps))
        data = self.frameQueue.get()

        while data != None:
            timestamp, frame = data
            text = str(timestamp) + "\n"
            timeF.write(text.encode('utf-8')) # 帧时间戳
            videoF.write(frame[:,:,:3])
            data = self.frameQueue.get()

        timeF.close()
        videoF.release()
        print("录制完成 -> %s | %s" %(videoN, timeN))

    # 捕获音频
    def audio(self,fileName,devIndex,rate,channels=1):
        # 以回调的方式读取才能获得ADC时间戳
        def callback(in_data, frame_count, time_info, status):
            wavF.writeframes(in_data) # 音频数据块
            wavTimeF.write(str(int(time_info["input_buffer_adc_time"]*1000))+"\n") # 音频时间戳
            # output=False时数据可以直接给b""，但是状态位还是要保持paContinue，如果是paComplete一样会停止录制
            return b"", pyaudio.paContinue

        audio = pyaudio.PyAudio()
        pformat = pyaudio.paInt16
        fileName += ".wav"

        wavF = wave.open(fileName, "wb")
        wavF.setnchannels(channels)
        wavF.setsampwidth(pyaudio.get_sample_size(pformat))
        wavF.setframerate(rate)

        wavTimeF = open(fileName+".txt","w")

        stream = audio.open(format=pformat,
                            channels=channels,
                            rate=rate,
                            input=True,
                            frames_per_buffer=2048,
                            input_device_index=devIndex,
                            stream_callback=callback)

        stream.start_stream()
        # 阻塞等待退出信号
        state = self.audioState.get()
        stream.stop_stream()

        stream.close()
        wavF.close()
        wavTimeF.close()

        audio.terminate()

    # 录制
    def record(self, fileName, aDevIndex, rate=44100, enc="avc1", suffix="mp4"):
        print("启动中...")
        vidSaver = multiprocessing.Process(target=self.save, args=(fileName,enc,suffix))
        audRecorder = multiprocessing.Process(target=self.audio, args=(fileName,aDevIndex,rate))
        if self.index != None:
            vidGrabber = multiprocessing.Process(target=self.grab_cam)
        elif self.area != None:
            vidGrabber = multiprocessing.Process(target=self.grab_scr)

        vidSaver.start()
        audRecorder.start()
        vidGrabber.start()
        vidSaver.join()
        audRecorder.join()
        vidGrabber.join()

  
if __name__ == "__main__":
    os.system("chcp 65001 && cls")
    task = VCap()
    # task.set_scr(index=1)

    index = int(input("读取第几个摄像头（从0开始）："))
    task.set_cam(index=index, height=1080, width=1920)

    index = int(input("显示在第几屏："))
    task.set_window(index=index, fullscreen=False)

    fileName=time.strftime("%Y-%m-%d_%H-%M-%S",time.localtime())
    enc = "avc1"#input("格式：")
    # 使用openh264-1.8.0-win64.dll会增加卡顿
    suffix = 'mp4'#input("后缀：")
    audIndex = int(input("录音设备索引："))
    rate = 44100#int(input("录音采样率："))
    task.record(fileName, audIndex, rate=rate, enc=enc, suffix=suffix)
    input()
