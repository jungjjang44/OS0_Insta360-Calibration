import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SyncCheck(Node):
    def __init__(self):
        super().__init__('sync_check')

        # 구독 QoS 설정
        # 카메라: RELIABLE-손실 없이 받는 쪽, 깊이 100, 과거 메시지 보존 X
        qos_cam   = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=100)
        
        # 라이다: 센서 스트림에 흔히 사용함. 깊이 20, 과거 메시지 보존 X
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=20)

        self.sub_cam = self.create_subscription(Image, '/image_raw', self.cam_cb, qos_profile=qos_cam)
        self.sub_lidar = self.create_subscription(PointCloud2, '/ouster/points', self.lidar_cb, qos_profile=qos_lidar)
        self.t_cam = self.t_lidar = None



    def cam_cb(self, msg):
        self.t_cam = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.print_diff()

    def lidar_cb(self, msg):
        self.t_lidar = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.print_diff()

    def print_diff(self):
        if self.t_cam and self.t_lidar:
            print(f"Δt (camera - lidar) = {self.t_cam - self.t_lidar:.6f} s")

def main(args=None):
    rclpy.init(args=args)
    node=SyncCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

# rclpy.init()
# rclpy.spin(SyncCheck())
