import cv2
import sys
import os

sys.path.append(f"{os.path.dirname(__file__)}/../optimal/scripts")
from lap_set_pk import lap_set

save_as_pictures = False
save_first_and_last_frame = True
last_frame = None

if save_as_pictures or save_first_and_last_frame:
    # 创建保存图片的文件夹（如果没有的话）
    output_folder = f'{lap_set.data_folder}/video/pictures'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)


# 读取视频文件
cap = cv2.VideoCapture(lap_set.video_file_path)


frame_count = 0

if save_first_and_last_frame and not save_as_pictures:
    ret, frame = cap.read()
    if ret:
        frame_count += 1
        cv2.imwrite(f'{output_folder}/{frame_count}.jpg', frame)
    else:
        print(f"cna't read the first frame!!!")
        exit()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    # cv2.imshow('Frame', frame)
    # cv2.waitKey(1)
    frame_count += 1
    if save_as_pictures:
        cv2.imwrite(f'{output_folder}/{frame_count}.jpg', frame)
    if save_first_and_last_frame:
        last_frame = frame

if save_first_and_last_frame and not save_as_pictures:
    cv2.imwrite(f'{output_folder}/{frame_count}.jpg', last_frame)


cap.release()
print("视频帧数(read n): ", frame_count)

print(f'视频帧数(cvcap): {cap.get(cv2.CAP_PROP_FRAME_COUNT)}')


with open(lap_set.video_time_stamp_file_path, 'r', encoding='utf-8') as file:
    line_count = sum(1 for _ in file)
print("time.perf_counter 时间戳帧数:", line_count)

with open(lap_set.video_time_stamp_cvcap_file_path, 'r', encoding='utf-8') as file:
    line_count = sum(1 for _ in file)
print("cv.capture 时间戳帧数:", line_count)
