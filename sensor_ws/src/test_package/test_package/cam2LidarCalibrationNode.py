#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import os
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

try:
    from utils import calibUtils
except ImportError:
    try:
        # 패키지명이 test_package 라는 가정(네 트리 구조 기준)
        from test_package.utils import calibUtils  # [FIX]
    except ImportError:
        raise

class cam2LidarCalibrationNode(Node):
    def __init__(self):
        super().__init__('cam_lidar_calib')
        self.declare_parameters('', [
            ('cam_topic', '/image_raw'), # 카메라 입력 토픽
            ('lidar_topic', '/ouster/points'), # 라이다 입력 토픽   
            ('YAML_PATH', 'src/test_package/config/os0_360cam.yaml'), # yaml 파일 위치
            ('yaml_pkg', 'test_package'),     
        ])
        cam_topic=self.get_parameter('cam_topic').value
        lidar_topic=self.get_parameter('lidar_topic').value
        yaml_path=self.get_parameter('YAML_PATH').value
        yaml_pkg=self.get_parameter('yaml_pkg').value 

        # 구독 QoS 설정
        # 카메라: RELIABLE-손실 없이 받는 쪽, 깊이 100, 과거 메시지 보존 X
        qos_cam   = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=100)
        
        # 라이다: 센서 스트림에 흔히 사용함. 깊이 20, 과거 메시지 보존 X
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=20)

        # 카메라
        self.sub_cam = self.create_subscription(Image, cam_topic, self.on_cam, qos_cam)
        self.sub_lidar = self.create_subscription(PointCloud2, lidar_topic, self.on_lidar, qos_lidar)

        if not os.path.isabs(yaml_path) and not os.path.exists(yaml_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory(str(yaml_pkg))
                # [FIX] share/<pkg>/config/<파일명>으로 조립
                candidate = os.path.join(pkg_share, 'config', os.path.basename(yaml_path))  # [FIX]
                if os.path.exists(candidate):
                    yaml_path = candidate
            except Exception:
                pass

        # YAML 로드
        raw = calibUtils.loadParameters(yaml_path)

        # [FIX] ROS2 스타일(YAML 계층)에 대응: cam_lidar_calib/ros__parameters 풀기
        if isinstance(raw, dict) and 'cam_lidar_calib' in raw and \
           isinstance(raw['cam_lidar_calib'], dict) and 'ros__parameters' in raw['cam_lidar_calib']:
            self.yamlParams = raw['cam_lidar_calib']['ros__parameters']
        else:
            self.yamlParams = raw  # 평평한 구조도 허용

        self.pc_np=None
        self.img=None
        self.width=self.yamlParams["parameters_cam"]["WIDTH"]
        self.height=self.yamlParams["parameters_cam"]["HEIGHT"]
        self.TransformMat=calibUtils.getTransFormMat(self.yamlParams)
        self.CameraMat=calibUtils.getCameraMat(self.yamlParams)

    # 
    def on_cam(self, msg: Image):
        self.last_cam_msg = msg
        # OpenCV 행렬로 변환
        self.img=np.frombuffer(msg.data,dtype=np.uint8).reshape(self.height,self.width,3)
        print(f"[CAM]  {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        # 라이다가 이미 들어왔다면 바로 처리
        if msg:
            self.calibrate()

    def on_lidar(self, msg: PointCloud2):
        pointList=[]
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pointList.append((p[0],p[1],p[2],1))
        self.pc_np=np.asarray(pointList,dtype=np.float32)
        self.last_lidar_msg = msg
        print(f"[LIDAR] {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        # 카메라가 이미 들어왔다면 바로 처리
        if msg:
            self.calibrate()

    def calibrate(self):
        pass



def main(args=None):
    rclpy.init(args=args)
    node=cam2LidarCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
