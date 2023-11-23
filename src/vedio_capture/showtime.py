import cv2
import sys
import os

sys.path.append(f"{os.path.dirname(__file__)}/../optimal/scripts")
from lap_set_pk import lap_set

# 打开视频文件
video_path = lap_set.vedio_file_path
cap = cv2.VideoCapture(video_path)

# 打开文本文件
txt_file_path = lap_set.vedio_time_stamp_file_path
with open(txt_file_path, 'r') as txt_file:
    lines = txt_file.readlines()

# 检查视频是否成功打开
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# 检查文本文件是否成功打开
if not lines:
    print("Error: Could not read lines from the text file.")
    exit()

# 输出视频文件设置
output_path = '/home/irobotcare/桌面/EX_Data/lap/test/time_video.avi' 
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'XVID'), 30, (frame_width, frame_height))

start_time = float(lines[0].strip())
frame_number = 0  # 记录帧数
while True:
    # 逐帧读取视频
    ret, frame = cap.read()

    # 检查是否成功读取帧
    if not ret:
        print("Error: Could not read frame.")
        break

    # 在每一帧上添加文本内容
    if frame_number < len(lines):
        delta_time = float(lines[frame_number].strip())-start_time-1
        text = f'{int(delta_time//3600):02d}:{(int(delta_time)%3600)//60:02d}:{int(delta_time%60):02d}:{int(delta_time%1*100):02d}'
        cv2.putText(frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        print(text)

    # 在这里可以对每一帧进行其他处理，如果需要的话

    # 将帧写入输出视频
    out.write(frame)

    # 按 'q' 键退出循环
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

    frame_number += 1

# 释放视频流和关闭窗口
cap.release()
out.release()
cv2.destroyAllWindows()
