import numpy as np

# 假设数据文件名为 "data.txt"
file_path = "/home/irobotcare/桌面/EX_Data/lap/test/Qualisys_data/rigid.txt"
rigid_name = 'rcm'

# 用于存储所有符合条件的xyz值
xyz_values = []

# 读取文件并处理
with open(file_path, "r") as file:
    for line in file:
        # 分割每行数据（以 tab 作为分隔符）
        data = line.strip().split('\t')
        
        # 如果数据行满足要求（即 name 为 rigid_name）
        if data[2] == rigid_name:
            # 获取 xyz 部分，格式为 "x,y,z"
            xyz = data[3].split(',')
            if xyz[0] !='nan':
                # 将 xyz 转换为浮动类型并存储
                xyz_values.append([float(xyz[0]), float(xyz[1]), float(xyz[2])])

# 计算 xyz 的平均值
xyz_array = np.array(xyz_values)
average_xyz = xyz_array.mean(axis=0)

# 输出结果
print(f"Average xyz for {rigid_name}: {average_xyz}")
