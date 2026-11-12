import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveThenStop(Node):
    def __init__(self):
        super().__init__('move_then_stop')

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.start_time = None

        self.timer = self.create_timer(
            0.1, self.timer_callback
        )

    def timer_callback(self):
        now = self.get_clock().now()
        msg = Twist()

        # 等待收到有效的仿真时间
        if now.nanoseconds == 0:
            self.publisher.publish(msg)
            return

        # 第一次收到有效时间时，才记录起点
        if self.start_time is None:
            self.start_time = now
            self.get_logger().info('开始前进，三秒后停车')

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed < 3.0:
            msg.linear.x = 0.1
        else:
            msg.linear.x = 0.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveThenStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()