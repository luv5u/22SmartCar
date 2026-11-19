import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_interfaces.msg import SensorData
from std_msgs.msg import Float32


class FilterNode(Node):
    def __init__(self):
        super().__init__('node_b_filter')
        self.declare_parameter('alpha', 0.3)
        self.add_on_set_parameters_callback(self.check_parameters)
        self.filtered = None
        self.publisher = self.create_publisher(Float32, '/processed_distance', 10)
        self.subscription = self.create_subscription(
            SensorData, '/sensor_data', self.on_reading, 10
        )

    def check_parameters(self, parameters):
        for param in parameters:
            if param.name == 'alpha':
                if param.type_ != Parameter.Type.DOUBLE or not 0.0 <= param.value <= 1.0:
                    return SetParametersResult(
                        successful=False, reason='alpha 必须是 0.0 到 1.0 之间的浮点数'
                    )
        return SetParametersResult(successful=True)

    def on_reading(self, msg):
        if msg.status != 0:
            self.get_logger().warning(f'跳过无效读数，status={msg.status}')
            return

        alpha = self.get_parameter('alpha').value
        if self.filtered is None:
            self.filtered = msg.distance
        else:
            self.filtered = alpha * msg.distance + (1.0 - alpha) * self.filtered

        self.publisher.publish(Float32(data=self.filtered))
        self.get_logger().info(
            f'alpha={alpha:.1f}  原始={msg.distance:.3f} m  '
            f'滤波={self.filtered:.3f} m',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
