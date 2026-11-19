import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def stamp_ns(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class SyncProbe(Node):
    def __init__(self):
        super().__init__('sync_probe')
        self.scan = Subscriber(
            self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data
        )
        self.image = Subscriber(
            self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.scan, self.image], queue_size=20, slop=0.05
        )
        self.sync.registerCallback(self.on_pair)
        self.count = 0

    def on_pair(self, scan, image):
        self.count += 1
        dt_ms = (stamp_ns(image) - stamp_ns(scan)) / 1_000_000
        self.get_logger().info(
            f'第 {self.count} 对 | 图像减激光 {dt_ms:+.2f} ms | '
            f'{scan.header.frame_id} → {image.header.frame_id}'
        )


if __name__ == '__main__':
    rclpy.init()
    node = SyncProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()