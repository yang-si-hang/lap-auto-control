'''
用于将rgb、bgr互换并保存
'''
import cv2
import numpy as np

# 打开视频文件
video = cv2.VideoCapture('/home/yiliao/870evo_1t/Experiment_Data/20230629/output_video.avi')

# 创建一个用于写入修正颜色后的帧的视频编写器
output = cv2.VideoWriter('/home/yiliao/870evo_1t/Experiment_Data/20230629/corrected_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))
cv2.namedWindow('figure', 0)
while True:
    # 读取一帧
    ret, frame = video.read()
    
    if not ret:
        break
    
    # 将颜色通道的顺序改变为RGB
    corrected_frame = frame[:, :, [2, 1, 0]]
    
    # 将修正后的帧写入输出视频
    output.write(corrected_frame)



    cv2.imshow('figure', corrected_frame)
        # outCamera.write(img_show)
    cv2.waitKey(1)

# 释放视频对象和视频编写器
video.release()
output.release()
