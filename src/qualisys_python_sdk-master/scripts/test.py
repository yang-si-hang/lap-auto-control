import os
import glob

# 文件夹路径
folder_path = '/home/irobotcare/桌面/EX_Data/lap/test/rigid_end_calibration/'

# 使用 glob 模块匹配文件路径
file_paths = glob.glob(os.path.join(folder_path, '*'))

# 提取文件名并添加到列表
file_names = [os.path.basename(file_path) for file_path in file_paths]
rigid_names = [i.replace('.txt','') for i in file_names]


# 打印文件名列表
print(file_paths)
print(file_names)
print(rigid_names)