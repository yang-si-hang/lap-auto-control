import cv2
import numpy as np
import threading
import queue

# 捕获线程的任务函数
def capture_thread(cap, buffer):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        buffer.put(frame)

# 保存线程的任务函数
def save_thread(output, buffer):
    frame_list = []
    while True:
        if buffer.empty():
            continue
        frame = buffer.get()
        frame_list.append(frame)
        if len(frame_list) >= 20:  # 保存每次触发时的10帧
            for f in frame_list:
                output.write(f)
            frame_list = []

# 打开USB摄像头
 # 打开USB摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('N', 'V', '1', '2'))
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(4, 1080)  # 图片宽度
cap.set(3, 1920)  # 图片宽度

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 获取摄像头的帧宽度和帧高度
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# 创建视频编写器对象，用于保存视频
output = cv2.VideoWriter('/home/yiliao/870evo_1t/Experiment_Data/20230629/output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30.0, (frame_width, frame_height))

# 创建缓冲区队列
buffer = queue.Queue(maxsize=200)

# 创建捕获线程
capture_t = threading.Thread(target=capture_thread, args=(cap, buffer))
capture_t.start()

# 创建保存线程
save_t = threading.Thread(target=save_thread, args=(output, buffer,))
save_t.start()

# 等待捕获线程完成
capture_t.join()

# 等待保存线程完成
save_t.join()

# 关闭视频编写器对象和摄像头
output.release()
cap.release()
