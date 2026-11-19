import math

import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorData


class SensorNode(Node):
    def __init__(self):
        super().__init__('node_a_sensor')
        self.publisher = self.create_publisher(SensorData, '/sensor_data', 10)
        self.frame = 0
        self.timer = self.create_timer(0.1, self.publish_reading)

    def publish_reading(self):
        msg = SensorData()
        msg.stamp = self.get_clock().now().to_msg()
        msg.unit = 'm'
        msg.distance = 0.5 + 0.3 * math.sin(2 * math.pi * self.frame / 60)
        msg.status = 0

        self.frame += 1
        if self.frame % 50 == 0:
            # 交替模拟超量程（1）和传感器错误（2）。
            msg.status = 1 if self.frame % 100 == 50 else 2

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
