import rclpy
from rclpy.node import Node
from sensor_interfaces.srv import TriggerAlarm


def main(args=None):
    rclpy.init(args=args)
    node = Node('alarm_server')

    def trigger(request, response):
        node.get_logger().info('[ALARM] 距离过近！')
        response.success = True
        return response

    node.create_service(TriggerAlarm, '/trigger_alarm', trigger)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
