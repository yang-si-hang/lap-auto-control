import os
import sys
import glob
import cv2
import torch
import gzip
import itertools
import random
import numpy
import math
import json
import pandas
import torchvision
from PIL import Image, ImageDraw
from torch import nn
from matplotlib import pyplot
from collections import defaultdict

# 图片大小
IMAGE_SIZE = (96, 96)
# 训练使用的数据集路径
DATASET_PATH = "test/test1211/my_clmp.npy"
DATASET_CSV_PATH = "test/test1211/clmp_keypoint.csv"
# 针对各张图片随机变形的数量
RANDOM_TRANSFORM_SAMPLES = 30

# 用于启用 GPU 支持
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class FaceLandmarkModel(nn.Module):
    """
    检测脸部关键点的模型 (基于 ResNet-18)
    针对图片输出:
    - 左眼中心点的 x 坐标
    - 左眼中心点的 y 坐标
    - 右眼中心点的 x 坐标
    - 右眼中心点的 y 坐标
    - 鼻尖的 x 坐标
    - 鼻尖的 y 坐标
    - 嘴巴左边角的 x 坐标
    - 嘴巴左边角的 y 坐标
    - 嘴巴右边角的 x 坐标
    - 嘴巴右边角的 y 坐标
    以上坐标均在 0 ~ 1 的范围内，表示相对图片长宽的位置
    """

    def __init__(self):
        super().__init__()
        # Resnet 的实现
        self.resnet = torchvision.models.resnet18(num_classes=256)
        # 支持黑白图片
        self.resnet.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)
        # 最终输出关键点的线性模型
        # 因为 torchvision 的 resnet 最终会使用一个 Linear，这里省略掉第一个 Lienar
        self.linear = nn.Sequential(
            nn.ReLU(inplace=True),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 10))

    def forward(self, x):
        tmp = self.resnet(x)
        y = self.linear(tmp)
        return y

    def detect_landmarks(self, images):
        """检测给出图片的关键点"""
        tensor_in = torch.stack([image_to_tensor(resize_image(img)) for img in images])
        tensor_out = self.forward(tensor_in.to(device))
        tensor_out = tensor_out.reshape(len(images), -1, 2)
        # 转换 -1 ~ 1 的坐标回绝对坐标
        size = torch.tensor(IMAGE_SIZE, dtype=torch.float).to(device)
        tensor_out = (tensor_out + 1) / 2 * size
        result = []
        for image, points in zip(images, tensor_out):
            points_mapped = []
            for point in points:
                points_mapped.append(map_point_to_original_image(point.tolist(), *image.size))
            result.append(points_mapped)
        return result


def save_tensor(tensor, path):
    """保存 tensor 对象到文件"""
    torch.save(tensor, gzip.GzipFile(path, "wb"))


def load_tensor(path):
    """从文件读取 tensor 对象"""
    return torch.load(gzip.GzipFile(path, "rb"))


def calc_resize_parameters(sw, sh):
    """计算缩放图片的参数"""
    sw_new, sh_new = sw, sh
    dw, dh = IMAGE_SIZE
    pad_w, pad_h = 0, 0
    if sw / sh < dw / dh:
        sw_new = int(dw / dh * sh)
        pad_w = (sw_new - sw) // 2  # 填充左右
    else:
        sh_new = int(dh / dw * sw)
        pad_h = (sh_new - sh) // 2  # 填充上下
    return sw_new, sh_new, pad_w, pad_h


def resize_image(img):
    """缩放图片，比例不一致时填充"""
    sw, sh = img.size
    sw_new, sh_new, pad_w, pad_h = calc_resize_parameters(sw, sh)
    img_new = Image.new("RGB", (sw_new, sh_new))
    img_new.paste(img, (pad_w, pad_h))
    img_new = img_new.resize(IMAGE_SIZE)
    return img_new


def image_to_tensor(img):
    """缩放并转换图片对象到 tensor 对象 (黑白)"""
    img = img.convert("L")  # 转换到黑白图片并缩放
    arr = numpy.asarray(img)
    t = torch.from_numpy(arr)
    t = t.unsqueeze(0)  # 添加通道
    t = t / 255.0  # 正规化数值使得范围在 0 ~ 1
    return t


def map_point_to_original_image(point, sw, sh):
    """把缩放后的坐标转换到缩放前的坐标"""
    x, y = point
    sw_new, sh_new, pad_w, pad_h = calc_resize_parameters(sw, sh)
    scale = IMAGE_SIZE[0] / sw_new
    x = int(x / scale - pad_w)
    y = int(y / scale - pad_h)
    x = min(max(0, x), sw - 1)
    y = min(max(0, y), sh - 1)
    return x, y


DefaultPointColors = ["#FF0000", "#FF00FF", "#00FF00", "#00FFFF", "#FFFF00"]


def draw_points(img, points, colors=None, radius=5):
    """在图片上描画关键点"""
    radius = math.fabs(points[2][0] - points[1][0]) * 0.5 + math.fabs(points[2][1] - points[1][1]) * 0.5
    cir_x = points[0][0] * 0.3 + points[1][0] * 0.3 + points[2][0] * 0.05 + points[3][0] * 0.05 + points[4][0] * 0.3
    cir_y = points[0][1] * 0.3 + points[1][1] * 0.3 + points[2][1] * 0.05 + points[3][1] * 0.05 + points[4][1] * 0.3

    cv2.circle(img, (int(cir_x), int(cir_y)), int(radius), (255, 255, 0), 1)
    # draw = ImageDraw.Draw(img)
    # colors = colors or DefaultPointColors
    # for index, point in enumerate(points):
    #     x, y = point
    #     color = colors[index] if index < len(colors) else colors[0]
    #     draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=color, width=1)


def generate_transform_theta(angle, scale, x_offset, y_offset, inverse=False):
    """
    计算变形参数
    angle: 范围 -math.pi ~ math.pi
    scale: 1 代表 100%
    x_offset: 范围 -1 ~ 1
    y_offset: 范围 -1 ~ 1
    inverse: 是否计算相反的变形参数 (默认计算把目标坐标转换为来源坐标的参数)
    """
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    if inverse:
        return (
            (cos_a * scale, sin_a * scale, -x_offset * cos_a * scale - y_offset * sin_a * scale),
            (-sin_a * scale, cos_a * scale, -y_offset * cos_a * scale + x_offset * sin_a * scale))
    else:
        return (
            (cos_a / scale, -sin_a / scale, x_offset),
            (sin_a / scale, cos_a / scale, y_offset))


def prepare_save_batch(batch, image_tensors, point_tensors):
    """准备训练 - 保存单个批次的数据"""
    # 连接所有数据
    # image_tensor 的维度会变为 数量,1,W,H
    # point_tensor 的维度会变为 数量,10 (10 = 5 个关键点的 x y 坐标)
    image_tensor = torch.cat(image_tensors, dim=0)
    image_tensor = image_tensor.unsqueeze(1)
    point_tensor = torch.cat(point_tensors, dim=0)
    point_tensor = point_tensor.reshape(point_tensor.shape[0], -1)

    # 切分训练集 (80%)，验证集 (10%) 和测试集 (10%)
    random_indices = torch.randperm(image_tensor.shape[0])
    training_indices = random_indices[:int(len(random_indices) * 0.8)]
    validating_indices = random_indices[int(len(random_indices) * 0.8):int(len(random_indices) * 0.9):]
    testing_indices = random_indices[int(len(random_indices) * 0.9):]
    training_set = (image_tensor[training_indices], point_tensor[training_indices])
    validating_set = (image_tensor[validating_indices], point_tensor[validating_indices])
    testing_set = (image_tensor[testing_indices], point_tensor[testing_indices])

    # 保存到硬盘
    save_tensor(training_set, f"dataPoint/data1211clmp/training_set.{batch}.pt")
    save_tensor(validating_set, f"dataPoint/data1211clmp/validating_set.{batch}.pt")
    save_tensor(testing_set, f"dataPoint/data1211clmp/testing_set.{batch}.pt")
    print(f"batch {batch} saved")


def prepare():
    """准备训练"""
    # 数据集转换到 tensor 以后会保存在 data 文件夹下
    if not os.path.isdir("dataPoint/data1211clmp"):
        os.makedirs("dataPoint/data1211clmp")

    # 加载原始数据集
    images_data = torch.from_numpy(numpy.load(DATASET_PATH)).float()
    images_csv = pandas.read_csv(DATASET_CSV_PATH, usecols=(
        "point1_x",
        "point1_y",
        "point2_x",
        "point2_y",
        "point3_x",
        "point3_y",
        "point4_x",
        "point4_y",
        "point5_x",
        "point5_y"
    ))

    # 原始数据的维度是 (W, H, 图片数量)，需要转换为 (图片数量, W, H)
    # images_data = images_data.permute(2, 0, 1)

    # 处理原始数据集
    batch = 0
    batch_size = 5  # 实际会添加 batch_size * (1 + RANDOM_TRANSFORM_SAMPLES) 到每个批次   ----------------原数据为20
    image_tensors = []
    point_tensors = []
    for image_data, image_row in zip(images_data, images_csv.values):
        # 读取图片参数
        w = image_data.shape[0]
        h = image_data.shape[1]
        assert w == IMAGE_SIZE[0]
        assert h == IMAGE_SIZE[1]
        size = torch.tensor((w, h), dtype=torch.float)
        points = torch.tensor(image_row, dtype=torch.float).reshape(-1, 2)
        points_std = points / size * 2 - 1  # 左上角 -1,-1 右下角 1,1
        # 正规化图片数据
        image_data = image_data / 255
        # 对图片进行随机变形
        thetas = []
        thetas_inverse = []
        for _ in range(RANDOM_TRANSFORM_SAMPLES):
            angle = math.pi * random.uniform(-1, 1)
            scale = random.uniform(0.30, 1.0)
            x_offset = random.uniform(-0.2, 0.2)
            y_offset = random.uniform(-0.2, 0.2)
            thetas.append(generate_transform_theta(angle, scale, x_offset, y_offset))
            thetas_inverse.append(generate_transform_theta(angle, scale, x_offset, y_offset, True))
        thetas_tensor = torch.tensor(thetas, dtype=torch.float)
        thetas_inverse_tensor = torch.tensor(thetas_inverse, dtype=torch.float)
        grid = nn.functional.affine_grid(
            thetas_tensor,
            torch.Size((RANDOM_TRANSFORM_SAMPLES, 1, w, h)),
            align_corners=False)
        images_transformed = nn.functional.grid_sample(
            image_data.repeat(RANDOM_TRANSFORM_SAMPLES, 1, 1)
                .reshape(RANDOM_TRANSFORM_SAMPLES, 1, w, h), grid)
        # 替换黑色的部分到随机颜色
        random_color = torch.rand(images_transformed.shape)
        zero_indices = images_transformed == 0
        images_transformed[zero_indices] = random_color[zero_indices]
        # 转换变形后的坐标
        points_std_with_one = torch.cat((points_std, torch.ones(points_std.shape[0], 1)), dim=1)
        points_std_transformed = points_std_with_one.matmul(thetas_inverse_tensor.permute(0, 2, 1))
        # 连接原图片和变形后的图片，原坐标和变形后的坐标
        images_cat = torch.cat((image_data.unsqueeze(0), images_transformed.squeeze(1)), dim=0)
        points_std_cat = torch.cat((points_std.unsqueeze(0), points_std_transformed), dim=0)
        points_cat = (points_std_cat + 1) / 2 * size
        # 测试变形后的图片与参数
        # for index, (img_data, points) in enumerate(zip(images_cat, points_cat)):
        #    img = Image.fromarray((img_data * 255).numpy()).convert("RGB")
        #    draw_points(img, points)
        #    img.save(f"test_{index}.png")
        # 保存批次
        image_tensors.append(images_cat)
        point_tensors.append(points_std_cat)
        if len(image_tensors) > batch_size:
            prepare_save_batch(batch, image_tensors, point_tensors)
            image_tensors.clear()
            point_tensors.clear()
            batch += 1
    # 保存剩余的批次
    if len(image_tensors) > 3:  # ---------------------------------------原数据为 10
        prepare_save_batch(batch, image_tensors, point_tensors)


def train():
    """开始训练"""
    # 创建模型实例
    model = FaceLandmarkModel().to(device)

    # 创建损失计算器
    def loss_function(predicted, actual):
        predicted_flatten = predicted.view(-1)
        actual_flatten = actual.view(-1)
        mask = torch.isnan(actual_flatten).logical_not()  # 用于跳过缺损的值
        return nn.functional.mse_loss(predicted_flatten[mask], actual_flatten[mask])

    # 创建参数调整器
    optimizer = torch.optim.Adam(model.parameters())

    # 记录训练集和验证集的正确率变化
    training_accuracy_history = []
    validating_accuracy_history = []

    # 记录最高的验证集正确率
    validating_accuracy_highest = -1
    validating_accuracy_highest_epoch = 0

    # 读取批次的工具函数
    def read_batches(base_path):
        for batch in itertools.count():
            path = f"{base_path}.{batch}.pt"
            if not os.path.isfile(path):
                break
            images_tensor, points_tensor = load_tensor(path)
            yield images_tensor.to(device), points_tensor.to(device)

    # 计算正确率的工具函数
    def calc_accuracy(actual, predicted):
        predicted_flatten = predicted.view(-1)
        actual_flatten = actual.view(-1)
        mask = torch.isnan(actual_flatten).logical_not()  # 用于跳过缺损的值
        diff = (predicted_flatten[mask] - actual_flatten[mask]).abs()
        return 1 - diff.mean().item()

    # 开始训练过程
    for epoch in range(1, 10000):
        print(f"epoch: {epoch}")

        # 根据训练集训练并修改参数
        # 切换模型到训练模式，将会启用自动微分，批次正规化 (BatchNorm) 与 Dropout
        model.train()
        training_accuracy_list = []
        for batch_index, batch in enumerate(read_batches("dataPoint/data1211clmp/training_set")):
            # 划分输入和输出
            batch_x, batch_y = batch
            # 计算预测值
            predicted = model(batch_x)
            # 计算损失
            loss = loss_function(predicted, batch_y)
            # 从损失自动微分求导函数值
            loss.backward()
            # 使用参数调整器调整参数
            optimizer.step()
            # 清空导函数值
            optimizer.zero_grad()
            # 记录这一个批次的正确率，torch.no_grad 代表临时禁用自动微分功能
            with torch.no_grad():
                training_batch_accuracy = calc_accuracy(batch_y, predicted)
            training_accuracy_list.append(training_batch_accuracy)
            print(f"epoch: {epoch}, batch: {batch_index}: batch accuracy: {training_batch_accuracy}")
        training_accuracy = sum(training_accuracy_list) / len(training_accuracy_list)
        training_accuracy_history.append(training_accuracy)
        print(f"training accuracy: {training_accuracy}")

        # 检查验证集
        # 切换模型到验证模式，将会禁用自动微分，批次正规化 (BatchNorm) 与 Dropout
        model.eval()
        validating_accuracy_list = []
        for batch in read_batches("dataPoint/data1211clmp/validating_set"):
            batch_x, batch_y = batch
            predicted = model(batch_x)
            validating_accuracy_list.append(calc_accuracy(batch_y, predicted))
            # 释放 predicted 占用的显存避免显存不足的错误
            predicted = None
        validating_accuracy = sum(validating_accuracy_list) / len(validating_accuracy_list)
        validating_accuracy_history.append(validating_accuracy)
        print(f"validating accuracy: {validating_accuracy}")

        # 记录最高的验证集正确率与当时的模型状态，判断是否在 20 次训练后仍然没有刷新记录
        if validating_accuracy > validating_accuracy_highest:
            validating_accuracy_highest = validating_accuracy
            validating_accuracy_highest_epoch = epoch
            save_tensor(model.state_dict(), "test/test1211/model.pt")
            print("highest validating accuracy updated")
        elif epoch - validating_accuracy_highest_epoch > 20:
            # 在 20 次训练后仍然没有刷新记录，结束训练
            print("stop training because highest validating accuracy not updated in 20 epoches")
            break

    # 使用达到最高正确率时的模型状态
    print(f"highest validating accuracy: {validating_accuracy_highest}",
          f"from epoch {validating_accuracy_highest_epoch}")
    model.load_state_dict(load_tensor("test/test1211/model.pt"))

    # 检查测试集
    testing_accuracy_list = []
    for batch in read_batches("dataPoint/testing_set"):
        batch_x, batch_y = batch
        predicted = model(batch_x)
        testing_accuracy_list.append(calc_accuracy(batch_y, predicted))
    testing_accuracy = sum(testing_accuracy_list) / len(testing_accuracy_list)
    print(f"testing accuracy: {testing_accuracy}")

    # 显示训练集和验证集的正确率变化
    pyplot.plot(training_accuracy_history, label="training")
    pyplot.plot(validating_accuracy_history, label="validing")
    pyplot.ylim(0, 1)
    pyplot.legend()
    pyplot.show()


def eval_model():
    """使用训练好的模型"""
    # 创建模型实例，加载训练好的状态，然后切换到验证模式
    model = FaceLandmarkModel().to(device)
    model.load_state_dict(load_tensor("test/test1211/model.pt"))
    model.eval()
    result_i = 0
    # 询问图片路径，并标记关键点
    # while True:
    #     try:
    # 打开图片
    # image_path = input("Image path: ")
    # if not image_path:
    #     continue
    # img = Image.open(image_path)
    for filename in glob.glob(r'D:\desktop\t_clmp\*.jpg'):
        # 检测关键点
        img = Image.open(filename)
        print(img)
        points = model.detect_landmarks([img])[0]
        src = cv2.imread(filename)
        # for point in points:
        #     print(point)
        # 输出图片与关键点
        draw_points(src, points)
        print(points)
        cv2.imwrite(r'D:\desktop\t_clmp\result\rclmp_result{}.jpg'.format(result_i), src)
        # img.save(r'D:\desktop\t_clmp\result\rclmp_result{}.jpg'.format(result_i))
        result_i += 1

    # print("saved successfully")
    # print()
    # except Exception as e:
    #     print("error:", e)


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print(f"Please run: {sys.argv[0]} prepare|train|eval")
        exit()

    # 给随机数生成器分配一个初始值，使得每次运行都可以生成相同的随机数
    # 这是为了让过程可重现，你也可以选择不这样做
    random.seed(0)
    torch.random.manual_seed(0)

    # 根据命令行参数选择操作
    operation = sys.argv[1]
    if operation == "prepare":
        prepare()
    elif operation == "train":
        train()
    elif operation == "eval":
        eval_model()
    else:
        raise ValueError(f"Unsupported operation: {operation}")


if __name__ == "__main__":
    eval_model()
#    main()
