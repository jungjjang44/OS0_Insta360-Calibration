#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 Insta360 saver: Odom 콜백이 "메인 루프"
# - /orb_slam3/odom(Odometry) 수신 시에만: 카메라 한 프레임 캡쳐 → JPEG 저장(+EXIF에 pose JSON) → /image_raw, /camera_info 퍼블리시
# - EXIF(UserComment)에: {"ros_timestamp_ns":..., "frame_id":..., "pose": {...}} 저장
# - piexif 필요: pip install piexif

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager

import cv2
import time
import threading
import os
import glob
import queue
import pathlib
import datetime
import json

from PIL import Image as PILImage
import piexif
from piexif.helper import UserComment as ExifUserComment


class Insta360Publisher(Node):
    def __init__(self):
        super().__init__('insta360_publisher')

        # --- Parameters ---
        self.declare_parameters("",[
            ("device","auto"),
            ("width",1920),
            ("height",1080),
            ("fps",30),
            ("fourcc","YUYV"),         # 리눅스 V4L2 기본값(필요시 MJPG로 바꿔도 됨)
            ("buffer_size",1),
            ("frame_id","insta360_frame"),
            ("camera_name","insta360"),
            ("camera_info_url",""),
            # 저장 파라미터
            ("save.enable", True),
            ("save.dir", "./frames"),
            ("save.jpeg_quality", 95),
        ])

        p = self.get_parameter
        req_dev = p("device").get_parameter_value().string_value
        self.dev = self._pick_camera_dev() if req_dev == "auto" else req_dev

        self.W    = int(p("width").value)
        self.H    = int(p("height").value)
        self.FPS  = int(p("fps").value)
        self.four = p("fourcc").get_parameter_value().string_value
        self.bufN = int(p("buffer_size").value)
        self.frame_id = p("frame_id").get_parameter_value().string_value

        # 저장 설정
        self.save_enable = bool(p("save.enable").value)
        self.save_dir    = p("save.dir").get_parameter_value().string_value or "./frames"
        self.jpeg_quality = int(p("save.jpeg_quality").value)

        if self.save_enable:
            pathlib.Path(self.save_dir).mkdir(parents=True, exist_ok=True)
            self.save_q = queue.Queue(maxsize=256)
            self.writer_running = True
            self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
            self.writer_thread.start()

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publishers
        self.pub_raw  = self.create_publisher(Image, "image_raw", qos)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", qos)

        # Bridge & CameraInfo
        self.bridge = CvBridge()
        self.cinfo_mgr = CameraInfoManager(
            self,
            p("camera_name").get_parameter_value().string_value,
            p("camera_info_url").get_parameter_value().string_value
        )
        self.cinfo_mgr.loadCameraInfo()

        # Camera open
        self.cap_lock = threading.Lock()
        self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera can't Open: {self.dev}")

        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.four))
        except Exception:
            pass
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_FPS,          self.FPS)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.bufN)
        except Exception:
            pass

        time.sleep(0.3)
        for _ in range(5):
            self.cap.read()

        # Odom subscriber (이 콜백이 메인 루프)
        self.create_subscription(Odometry, "/orb_slam3/odom", self.pose_callback, 10)

        self.get_logger().info("Insta360 saver ON (triggered by /orb_slam3/odom)")

    # pick device by-id
    def _pick_camera_dev(self)->str:
        for path in sorted(glob.glob("/dev/v4l/by-id/*Insta*360*index0")):
            if os.path.exists(path):
                return path
        for path in sorted(glob.glob("/dev/v4l/by-id/*Insta*360*index*")):
            if os.path.exists(path):
                return path
        return "/dev/video0"

    # 안전 재오픈
    def _reopen(self):
        with self.cap_lock:
            try:
                self.cap.release()
            except Exception:
                pass
            time.sleep(0.1)
            self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
            if self.cap.isOpened():
                try:
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.four))
                except Exception:
                    pass
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
                self.cap.set(cv2.CAP_PROP_FPS,          self.FPS)
                try:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.bufN)
                except Exception:
                    pass
                time.sleep(0.2)
                for _ in range(5):
                    self.cap.read()
                self.get_logger().info("Camera reopened")
                return True
            else:
                self.get_logger().warn("Camera reopen failed")
                return False

    # -------- Odom 콜백(메인 루프 역할) --------
    def pose_callback(self, msg: Odometry):
        # self.get_logger().info("Pose callback ON!")
        # 1) odom 타임스탬프 사용
        stamp = msg.header.stamp
        ts_ns = Time.from_msg(stamp).nanoseconds

        # 2) 프레임 캡쳐(실패 시 재오픈 1회 시도)
        with self.cap_lock:
            ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("cap.read() failed, trying reopen...")
            if not self._reopen():
                return
            with self.cap_lock:
                ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn("cap.read() failed after reopen, skip this odom")
                return

        # 3) /image_raw, /camera_info 퍼블리시 (stamp = odom의 stamp)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id

        info = self.cinfo_mgr.getCameraInfo()
        info.header = img_msg.header

        self.pub_raw.publish(img_msg)
        self.pub_info.publish(info)

        # 4) 저장(요청대로 JPEG + EXIF에 pose JSON)
        if self.save_enable:
            pose_dict = {
                "header": {
                    "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
                    "frame_id": msg.header.frame_id
                },
                "child_frame_id": msg.child_frame_id,
                "pose": {
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z
                    },
                    "orientation": {
                        "x": msg.pose.pose.orientation.x,
                        "y": msg.pose.pose.orientation.y,
                        "z": msg.pose.pose.orientation.z,
                        "w": msg.pose.pose.orientation.w
                    }
                },
            }
            base = f"{ts_ns}_{self.frame_id}"
            img_path = os.path.join(self.save_dir, base + ".jpg")
            payload = {
                "ros_timestamp_ns": ts_ns,
                "frame_id": self.frame_id,
                "pose": pose_dict
            }
            try:
                # writer 스레드로 넘김 (프레임 복사)
                self.save_q.put_nowait((img_path, frame.copy(), payload))
            except queue.Full:
                # 실시간성 우선, 드롭
                pass

    # -------- 비동기 저장 스레드: JPEG + EXIF(UserComment=JSON) --------
    def _writer_loop(self):
        while getattr(self, "writer_running", False) or (hasattr(self, "save_q") and not self.save_q.empty()):
            try:
                img_path, frame, payload = self.save_q.get(timeout=0.2)
            except Exception:
                continue
            try:
                ts_ns = int(payload["ros_timestamp_ns"])
                dt = datetime.datetime.utcfromtimestamp(ts_ns / 1e9)
                exif_dt = dt.strftime("%Y:%m:%d %H:%M:%S")
                subsec  = f"{ts_ns % 1_000_000_000:09d}"

                # OpenCV BGR -> PIL RGB
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_img = PILImage.fromarray(rgb)

                # EXIF(UserComment에 JSON) 구성
                user_comment_text = json.dumps(payload, ensure_ascii=False)
                zeroth_ifd = { piexif.ImageIFD.Software: "ROS2 Insta360 saver" }
                exif_ifd  = {
                    piexif.ExifIFD.DateTimeOriginal:    exif_dt,
                    piexif.ExifIFD.SubSecTimeOriginal:  subsec,
                    piexif.ExifIFD.UserComment:         ExifUserComment.dump(user_comment_text, encoding="unicode"),
                }
                exif_dict  = {"0th": zeroth_ifd, "Exif": exif_ifd, "GPS": {}, "1st": {}}
                exif_bytes = piexif.dump(exif_dict)

                pil_img.save(img_path, format="JPEG", quality=self.jpeg_quality, exif=exif_bytes)
            except Exception as e:
                try:
                    self.get_logger().warn(f"Save failed: {img_path} ({e})")
                except Exception:
                    pass

    def shutdown(self):
        try:
            if getattr(self, "save_enable", False):
                self.writer_running = False
                if hasattr(self, "writer_thread") and self.writer_thread.is_alive():
                    self.writer_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if hasattr(self, "cap") and self.cap is not None:
                self.cap.release()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Insta360Publisher()
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
