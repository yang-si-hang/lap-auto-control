# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage:
    $ python path/to/detect.py --weights yolov5s.pt --source 0  # webcam
                                                             img.jpg  # image
                                                             vid.mp4  # video
                                                             path/  # directory
                                                             path/*.jpg  # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path
from utils import datasets
import cv2
import numpy
import torch
import torch.backends.cudnn as cudnn
import matplotlib.pyplot as plt

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
myselect_index = 0

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
import keyPoints_find
from PIL import Image


class Model(torch.nn.Module):

    def __init__(self):
        super(Model, self).__init__()

        self.sigmoid = torch.nn.Sigmoid()
        self.linear1 = torch.nn.Linear(8, 12)

        self.linear2 = torch.nn.Linear(12, 1)

    def forward(self, model_x):
        model_x = self.sigmoid(self.linear1(model_x))
        y_pred = self.sigmoid(self.linear2(model_x))
        return y_pred


def data_frame(every_row):
    # Ligation，scissors，Right，Separating，ultrasound，Intestinal，Bipolar
    tool_class = [0, 0, 0, 0, 0, 0, 0]
    tool_postion_x = [None, None, None, None, None, None, None]
    tool_postion_y = [None, None, None, None, None, None, None]
    tool_postion_area = [None, None, None, None, None, None, None]
    tool_tip = [False, False, False, False, False, False, False]

    row_index = 4
    while row_index < len(every_row):
        if str(every_row[row_index]) == "nan":
            break
        elif str(every_row[row_index]).startswith("Ligation"):
            i = 0
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])
        elif str(every_row[row_index]).startswith("scissors"):
            i = 1
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])
        elif str(every_row[row_index]).startswith("Right"):
            i = 2
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])
        elif str(every_row[row_index]).startswith("Separating"):
            i = 3
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])
        elif str(every_row[row_index]).startswith("ultrasound"):
            i = 4
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])
        elif str(every_row[row_index]).startswith("Intestinal"):
            i = 5
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])
        elif str(every_row[row_index]).startswith("Bipolar"):
            i = 6
            tool_class[i] = 1
            tool_postion_x[i] = float(every_row[row_index - 4])
            tool_postion_y[i] = float(every_row[row_index - 3])
            tool_postion_area[i] = float(every_row[row_index - 2])
            tool_tip[i] = bool(every_row[row_index - 1])

        row_index += 5

    frame = [tool_class, tool_postion_x, tool_postion_y, tool_postion_area, tool_tip]
    return frame


def frame_to_row(every_row):
    weights_tools = [0.3, 0.25, 0.2, 0.15, 0.05, 0.05, 0.05]
    row = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    i = 0
    judge = 0
    while i < 7:
        if every_row[0][i] == 1:
            judge += 1
            row[judge * 4 - 4] = weights_tools[i] * every_row[1][i]
            row[judge * 4 - 3] = weights_tools[i] * every_row[2][i]
            row[judge * 4 - 2] = weights_tools[i] * every_row[3][i]
            if every_row[4][i]:
                row[judge * 4 - 1] = weights_tools[i]
            else:
                row[judge * 4 - 1] = 0

        if judge == 2:
            break

        i += 1

    return row



@torch.no_grad()
def run(weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        source=ROOT / 'my_date/rightFrame/images/',  # file/dir/URL/glob, 0 for webcam
        imgsz=640,  # inference size (pixels)
        conf_thres=0.25,  # confidence threshold 置信度阈值
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image  最大检测/形象
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features  可视化功能
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness= 3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        ):
    num_tip = 0
    num_tool = 0
    tip_conf = 0.00
    label_sort = ['Ligation', 'scissors', 'Right', 'Separating', 'ultrasound', 'Intestinal', 'Bipolar']
    # 加载8数组线性化模型
    scal = [447.3,189.3,328860,0.3,205.425,145.6,77004,0.2]
    my_keyframe_detect_model = torch.load("features/classification/full_best_0.94_0.93_0.94.pt")
    my_keyframe_detect_model.eval()


    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)

    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
    if is_url and is_file:
        source = check_file(source)  # download


    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # 加载嵌入的检测模型,先试一下先啊-------------------------------------------------------------------------------
    weights_tip = ROOT / 'runs/train/boss_tip1230/weights/best.pt'
    half_tip = False

    model_tip = DetectMultiBackend(weights_tip, device=device, dnn=dnn)
    stride_tip, names_tip, pt_tip, jit_tip, onnx_tip, engine_tip = model_tip.stride, model_tip.names, model_tip.pt, model_tip.jit, model_tip.onnx, model_tip.engine

    # ----------------------------------------------------------------------------------------------------------

    # Half
    half &= (pt or engine) and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
    if pt:
        model.model.half() if half else model.model.float()

    # Half_tip----------------------------------------------
    half_tip &= (pt_tip or engine_tip) and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
    if pt_tip:
        model_tip.model.half() if half_tip else model_tip.model.float()

    # Dataloader 这里是加载数据，嵌入的模型不用加载数据
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt and not jit)
        bs = len(dataset)  # batch_size
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt and not jit)
        bs = 1  # batch_size
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1, 3, *imgsz), half=half)  # warmup
    model_tip.warmup(imgsz=(1,3,*imgsz),half=half_tip)

    dt, seen = [0.0, 0.0, 0.0], 0
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        # print(im.shape)
        # print(im0s.shape)
        # image_chw = numpy.transpose(im, (1, 2, 0))
        # cv2.imshow("test0", image_chw)
        # cv2.imshow("test", im0s)
        # cv2.waitKey(0)
        im = torch.from_numpy(im).to(device)
        im = im.half() if half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        pred = model(im, augment=augment, visualize=visualize)

        # print("pred = ",pred)

        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3
        # print(len(pred))
        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop 之前是if save_crop else +++++++++++修改过+++++++++++++++++=
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                csv_data =[]
                temp_row = []
                for *xyxy, conf, cls in reversed(det):
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or save_crop or view_img:  # Add bbox to image
                        num_tool += 1
                        # --------------------------  截取检测区域  ---------------------------------------------
                        temp_x1 = int(xyxy[0].item())
                        temp_y1 = int(xyxy[1].item())
                        temp_x2 = int(xyxy[2].item())
                        temp_y2 = int(xyxy[3].item())
                        csv_data.append(str((temp_x1+temp_x2)/2))
                        csv_data.append(str((temp_y1+temp_y2)/2))
                        csv_data.append(str((temp_y2-temp_y1)*(temp_x2-temp_x1)))
                        temp_src = imc[temp_y1:temp_y2, temp_x1:temp_x2]  # 截取box区域  裁剪【y1,y2：x1,x2】
                        # Padded resize
                        img_tip = datasets.letterbox(temp_src, 640, 32, True)[0]

                        # Convert
                        img_tip = img_tip.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
                        img_tip = numpy.ascontiguousarray(img_tip)
                        img_tip = torch.from_numpy(img_tip).to(device)
                        img_tip = img_tip.half() if half else img_tip.float()  # uint8 to fp16/32
                        img_tip /= 255  # 0 - 255 to 0.0 - 1.0
                        if len(img_tip.shape) == 3:
                            img_tip = img_tip[None]  # expand for batch dim

                        pred_tip = model_tip(img_tip, augment=augment, visualize=visualize)
                        # print(pred_tip)
                        # 交并比与极大值抑制
                        pred_tip = non_max_suppression(pred_tip, 0.10, iou_thres, classes, agnostic_nms, max_det=max_det)
                        for i_tip, det_tip in enumerate(pred_tip):  # per image
                            # -------------------写入判断尖端的结果
                            tip_conf = 0.00
                            if len(det_tip):
                                num_tip += 1
                                csv_data.append("True")
                                det_tip[:, :4] = scale_coords(im.shape[2:], det_tip[:, :4], temp_src.shape).round()
                                for *xyxy_tip, conf_tip, cls_tip in reversed(det_tip):
                                    tip_conf = conf_tip.item()

                            else:
                                csv_data.append("False")

                        # --------------------添加训练好的状态模型，切换到验证状态-------------------------------

                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}_tip_{tip_conf:.2f}')
                        csv_data.append(str(label))

                        annotator.box_label(xyxy, label, color=colors(c, True))

                        if save_crop:
                            save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
                # ---------------------给数据排序----------------------------
                # 排序的顺序是：结扎夹>剪刀>直角分离钳>分离钳>超声刀>肠钳>双极电凝
                # 前缀的顺序是：Ligation，scissors，Right，Separating，ultrasound，Intestinal，Bipolar

                len_csv_data = len(csv_data)
                while len(temp_row) != len_csv_data:
                    for temp_row_index in range(0, 7):
                        temp_row_i = 4
                        while temp_row_i < len(csv_data):
                            if str(csv_data[temp_row_i]).startswith(label_sort[temp_row_index]):  # 结扎夹前缀
                                temp_row.append(csv_data[temp_row_i-4])
                                temp_row.append(csv_data[temp_row_i-3])
                                temp_row.append(csv_data[temp_row_i-2])
                                temp_row.append(csv_data[temp_row_i-1])
                                temp_row.append(csv_data[temp_row_i])
                            temp_row_i += 5
                # 取前两个器械信息并权重处理后加入到key_frame_row中，之后再将此序列输入到判定关键帧模型中
                # "Instrument1_x_score", "Instrument1_y_score","Instrument1_area_score", "Tip1_score",
                # "Instrument2_x_score", "Instrument2_y_score","Instrument2_area_score", "Tip2_score",
                # 权重顺序是：结扎夹>剪刀>直角分离钳>分离钳>超声刀>肠钳>双极电凝
                #           0.3 >0.25>   02  > 0.15 >0.05>0.05>0.05
                # 关键帧判定代码！
                frame_data = data_frame(temp_row)
                key_frame_row = frame_to_row(frame_data)
                i_key_frame_row = 0
                while i_key_frame_row < 8:
                    key_frame_row[i_key_frame_row] = key_frame_row[i_key_frame_row]/scal[i_key_frame_row]
                    i_key_frame_row += 1
                frame_judge = torch.FloatTensor(numpy.array([key_frame_row]))
                pred_judge = my_keyframe_detect_model(frame_judge)
                print("this keyFrame conf:", pred_judge.item())
                if pred_judge.item() >= 0.5:
                    print("oh my god! this is what I want!")
                else:
                    print("NO No No, I don't need it.")

            # Print time (inference-only)

            LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

            # Stream results
            im0 = annotator.result()
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':

                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path += '.mp4'
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights)  # update model (to fix SourceChangeWarning)

    print("识别的尖端数：", num_tip)
    print("识别的器具数：", num_tool)



def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'runs/train/boss_test_v1/weights/best.pt',
                        help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'my_date/test_single_image', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect/key_frame_detect', help='save results to project/name')
    parser.add_argument('--name', default='key_frame_detect_sum', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(FILE.stem, opt)
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
