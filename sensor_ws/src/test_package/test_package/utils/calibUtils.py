#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import time
import yaml
import os
from numpy.linalg import inv

def loadParameters(yaml_path:str):
    with open(yaml_path,'r',encoding='utf-8') as f:
        params=yaml.safe_load(f)
    return params

# Rotation Matrix 생성 - deg/rad 확인할 것
def getRotMat(RPY):
    cosR = math.cos(RPY[0])
    cosP = math.cos(RPY[1])
    cosY = math.cos(RPY[2])
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])
    rotRoll = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
    rotPitch = np.array([cosP,0,sinP, 0,1,0, -sinP,0,cosP]).reshape(3,3)
    rotYaw = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))
    return rotMat

# 카메라-라이다 위치/회전 변환 관계 구하기
def getTransFormMat(params):
    params_cam = params["parameters_cam"]
    params_lidar = params["parameters_lidar"]
    lidarPosition=np.array([params_lidar[i] for i in ["X","Y","Z"]])
    camPosition=np.array([params_cam[i] for i in ["X","Y","Z"]])
    lidarRPY=np.array([params_lidar[i] for i in ["ROLL","PITCH","YAW"]])
    camRPY=np.array([params_cam[i] for i in ["ROLL","PITCH","YAW"]])

    # 회전 행렬 구하기
    camRot=getRotMat(camRPY)
    camTransl=np.array([camPosition])
    TrCamtoVehicle=np.concatenate((camRot,camTransl.T),axis=1)
    TrCamtoVehicle=np.insert(TrCamtoVehicle,3,values=[0,0,0,1],axis=0)

    # 라이다 원점 기준
    lidarRot=getRotMat(lidarRPY)
    lidarTransl=np.array([lidarPosition])
    TrLidartoVehicle=np.concatenate((lidarRot,lidarTransl.T),axis=1)
    TrLidartoVehicle=np.insert(TrLidartoVehicle,3,values=[0,0,0,1],axis=0)    

    invTr=inv(TrCamtoVehicle)
    TrLidartoCam=invTr.dot(TrLidartoVehicle).round(6)

    return TrLidartoCam

# 카메라 내부 파라미터 행렬화
def getCameraMat(params):
    params_cam = params["parameters_cam"]

    focalLength=params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    principalX=params_cam["WIDTH"]/2
    principalY=params_cam["HEIGHT"]/2    
    CameraMat=np.array([focalLength,0.,principalX,0,focalLength,principalY,0,0,1]).reshape(3,3)

    return CameraMat

