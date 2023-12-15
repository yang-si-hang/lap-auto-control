# import cv2

# # 打开PCIe采集卡
# cap = cv2.VideoCapture("/dev/video1", cv2.CAP_V4L2)  # 请根据实际情况替换为你的PCIe采集卡的设备文件

# # 检查采集卡是否成功打开
# if not cap.isOpened():
#     print("无法打开采集卡")
#     exit()

# # 读取视频帧
# while True:
#     ret, frame = cap.read()

#     # 在这里进行你的视频处理逻辑
#     if ret:
#         cv2.imshow("Video", frame)

#     # 检测按键，按 'q' 键退出循环
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # 释放资源
# cap.release()
# cv2.destroyAllWindows()

# ==================================================================================
import cv2

# 打开PCIe采集卡
cap = cv2.VideoCapture("pci://02:00.0")  # 根据你的采集卡地址进行更改

# 检查采集卡是否成功打开
if not cap.isOpened():
    print("无法打开采集卡")
    exit()

# 读取视频帧
while True:
    ret, frame = cap.read()

    # 在这里进行你的视频处理逻辑
    if ret:
        cv2.imshow("Video", frame)

    # 检测按键，按 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
