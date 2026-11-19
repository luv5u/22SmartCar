import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from sync_probe import SyncProbe


class ProjectionProbe(SyncProbe):
    def __init__(self):
        super().__init__()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.bridge = CvBridge()
        self.saved = False

    def on_pair(self, scan, image):
        if self.saved:
            return

        try:
            tf = self.buffer.lookup_transform(
                image.header.frame_id,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
            )
        except TransformException as exc:
            self.get_logger().warning(f'等待 TF：{exc}', throttle_duration_sec=2.0)
            return

        canvas = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8').copy()
        height, width = canvas.shape[:2]
        offset = tf.transform.translation
        visible = 0

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # 本题刚验证了相对旋转为单位四元数，因此只需加平移。
            x = distance * math.cos(angle) + offset.x
            y = distance * math.sin(angle) + offset.y
            z = offset.z

            if x <= 0:
                continue

            # camera_link 的 x 向前、y 向左、z 向上；像素 u 向右、v 向下。
            u = round(320.0 - 500.0 * y / x)
            v = round(240.0 - 500.0 * z / x)
            if 0 <= u < width and 0 <= v < height:
                cv2.circle(canvas, (u, v), 2, (0, 0, 255), -1)
                visible += 1

        if visible:
            self.saved = cv2.imwrite('projection_first.png', canvas)
            if self.saved:
                self.get_logger().info(f'已保存 projection_first.png，图内有 {visible} 个激光点')
            else:
                self.get_logger().error('图片写入失败，请检查当前目录的写入权限')
        else:
            self.get_logger().warning('这一帧没有激光点落进图像', throttle_duration_sec=2.0)


if __name__ == '__main__':
    rclpy.init()
    node = ProjectionProbe()
    try:
        while rclpy.ok() and not node.saved:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()