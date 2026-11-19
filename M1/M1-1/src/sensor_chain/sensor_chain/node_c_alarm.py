from time import monotonic

import rclpy
from rclpy.node import Node
from sensor_interfaces.srv import TriggerAlarm
from std_msgs.msg import Float32


class AlarmNode(Node):
    def __init__(self):
        super().__init__('node_c_alarm')
        self.declare_parameter('alarm_threshold', 0.3)
        self.declare_parameter('service_timeout', 1.0)
        self.client = self.create_client(TriggerAlarm, '/trigger_alarm')
        self.subscription = self.create_subscription(
            Float32, '/processed_distance', self.on_distance, 10
        )
        self.pending = None
        self.deadline = 0.0
        self.below_threshold = False
        self.timer = self.create_timer(0.05, self.check_response)

    def on_distance(self, msg):
        threshold = self.get_parameter('alarm_threshold').value
        below = msg.data < threshold

        # 每次进入过近区间时报警一次，避免同一次靠近连续触发。
        if below and not self.below_threshold and self.pending is None:
            self.get_logger().info(f'距离 {msg.data:.3f} m，触发报警请求')
            self.deadline = monotonic() + self.get_parameter('service_timeout').value
            self.pending = self.client.call_async(TriggerAlarm.Request())
        self.below_threshold = below

    def check_response(self):
        future = self.pending
        if future is None:
            return

        if future.done():
            self.pending = None
            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().warning(f'报警服务调用失败：{exc}')
                return
            if response.success:
                self.get_logger().info('报警服务已确认')
            else:
                self.get_logger().warning('报警服务返回失败')
        elif monotonic() >= self.deadline:
            self.client.remove_pending_request(future)
            future.cancel()
            self.pending = None
            self.get_logger().warning('报警服务超时')


def main(args=None):
    rclpy.init(args=args)
    node = AlarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
