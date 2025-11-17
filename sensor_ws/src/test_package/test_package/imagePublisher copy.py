#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# CAM publisher
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy,QoSDurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager

# 이미지 및 기타 관련
import cv2,time,threading,signal
import os,glob


class Insta360Publisher(Node):
    def __init__(self):
        super().__init__('insta360_publisher')

        # --- Parameter ---
        self.declare_parameters("",[
            ("device","auto"),
            ("width",1920),
            ("height",1080),
            ("fps",30),
            ("fourcc","MJPG"),
            ("buffer_size",1),
            ("crop.mode","bottom_half"),
            ("crop.offset",0),
            ("crop.rect",[0, 0, 0, 0]),
            ("frame_id","insta360_frame"),
            ("camera_name","insta360"),
            ("camera_info_url",""),
            ("publish_compressed",False)
        ])
        
        p=self.get_parameter
        req_dev=p("device").get_parameter_value().string_value
        self.dev=self._pick_camera_dev()
        self.W    = int(p("width").value)
        self.H    = int(p("height").value)
        self.FPS  = int(p("fps").value)
        self.four = p("fourcc").get_parameter_value().string_value
        self.bufN = int(p("buffer_size").value)
        self.crop_mode   = p("crop.mode").get_parameter_value().string_value
        self.crop_offset = int(p("crop.offset").value)
        self.crop_rect   = [int(v) for v in p("crop.rect").value]
        self.frame_id    = p("frame_id").get_parameter_value().string_value
        self.publish_compressed = bool(p("publish_compressed").value)

        # QoS (Sensor Data)
        qos=QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE, # 최신 데이터만 송수신
            history=QoSHistoryPolicy.KEEP_LAST, # 최근 N개의 메시지만 버퍼에 저장. 메모리 부담 감소 목적
            depth=5, # 최근 메시지 5개까지만 버퍼에 보관
        )

        # Publisher
        self.pub_raw=self.create_publisher(Image,"image_raw",qos)
        self.pub_info=self.create_publisher(CameraInfo,"camera_info",qos)
        if self.publish_compressed:
            self.get_logger().info("image_view or rviz -> /image_raw/compressed")
        
        # Camera Info
        self.bridge=CvBridge()
        self.cinfo_mgr=CameraInfoManager(self,p("camera_name").get_parameter_value().string_value,
                                         p("camera_info_url").get_parameter_value().string_value)
        self.cinfo_mgr.loadCameraInfo()

        # 캡쳐 시작
        self.cap=cv2.VideoCapture(self.dev,cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera can't Open:{self.dev}")
        
        # 카메라 설정
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*self.four))
        except Exception:
            pass
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.H)
        self.cap.set(cv2.CAP_PROP_FPS,self.FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,self.bufN)

        time.sleep(0.4)
        for _ in range(5):
            self.cap.read()
        self.running=True
        self.thread=threading.Thread(target=self.loop)
        self.thread.start()
        self.get_logger().info(f"Insta360 Webcam Mode ON")
    
    # Insta360 Device select
    def _pick_camera_dev(self)->str:
        for path in sorted(glob.glob("/dev/v4l/by-id/*Insta*360*index0")):
            if os.path.exists(path):
                return path
        for path in sorted(glob.glob("/dev/v4l/by-id/*Insta*360*index*")):
            if os.path.exists(path):
                return path
        return "/dev/video0"        

    # Crop
    def apply_crop(self,frame):
        h,w=frame.shape[:2]
        y=h//2+self.crop_offset
        y=max(0,min(y,h-1))
        return frame[y:,:]

    # 메인 루프
    def loop(self):
        stamp=self.get_clock().now()
        while rclpy.ok() and self.running:
            ok,frame=self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            frame=self.apply_crop(frame)

            # message convert
            msg=self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")
            msg.header.stamp=self.get_clock().now().to_msg()
            msg.header.frame_id=self.frame_id # insta360_frame

            # Camera Info
            info=self.cinfo_mgr.getCameraInfo()
            info.header=msg.header

            self.pub_raw.publish(msg)
            self.pub_info.publish(info)
        self.get_logger().info("main loop exit")
    
    def shutdown(self):
        self.running=False
        try:
            if self.thread.is_alive():
                self.thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node=Insta360Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
