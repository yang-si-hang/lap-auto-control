import json
import os.path
import cv2
import h5py
import numpy as np
import glob

mtx_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/temp/mtx.csv'
dist_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/temp/dist.csv'
T_rob_camera_path =  f'{os.path.dirname(__file__)}/../data/Camera_Calibration/temp/camera_tool.csv'

mtx = np.loadtxt(mtx_path)
dist = np.loadtxt(dist_path)
T_rob_camera = np.loadtxt(T_rob_camera_path)