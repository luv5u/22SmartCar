import math
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from sync_probe import SyncProbe, stamp_ns


class ProjectionStream(SyncProbe):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data
        )
        self.overlay_pub = self.create_publisher(Image, '/projection/image', 5)
        self.output_dir = Path('results/projection')
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.saved = 0
        self.last_saved_ns = None

    def on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.camera_info = msg

    def on_pair(self, scan, image):
        info = self.camera_info
        if info is None or (info.width, info.height) != (image.width, image.height):
            return

        try:
            transform = self.buffer.lookup_transform(
                image.header.frame_id, scan.header.frame_id,
                Time.from_msg(scan.header.stamp)
            ).transform
        except TransformException as exc:
            self.get_logger().warning(f'等待 TF：{exc}', throttle_duration_sec=2.0)
            return

        canvas = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8').copy()
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        t, q = transform.translation, transform.rotation

        # LaserScan 的点都在雷达的 z=0 平面，只需要旋转矩阵的前两列。
        col_x = (1 - 2*(q.y*q.y + q.z*q.z),
                 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y))
        col_y = (2*(q.x*q.y - q.w*q.z),
                 1 - 2*(q.x*q.x + q.z*q.z), 2*(q.y*q.z + q.w*q.x))

        visible = 0
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            lx, ly = distance * math.cos(angle), distance * math.sin(angle)
            x = col_x[0]*lx + col_y[0]*ly + t.x
            y = col_x[1]*lx + col_y[1]*ly + t.y
            z = col_x[2]*lx + col_y[2]*ly + t.z
            if x <= 0:
                continue

            # 题目中的 camera_link 采用 x 向前、y 向左、z 向上。
            u, v = round(cx - fx*y/x), round(cy - fy*z/x)
            if 0 <= u < image.width and 0 <= v < image.height:
                fraction = (distance - scan.range_min) / (scan.range_max - scan.range_min)
                blue = round(255 * fraction)
                cv2.circle(canvas, (u, v), 2, (blue, 0, 255 - blue), -1)
                visible += 1

        overlay = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        overlay.header = image.header
        self.overlay_pub.publish(overlay)
        self.count += 1

        # 用图像消息时间间隔选帧，回放速度改变也能取到分开的时刻。
        now_ns = stamp_ns(image)
        due = self.last_saved_ns is None or now_ns - self.last_saved_ns >= 500_000_000
        if visible and self.saved < 3 and due:
            path = self.output_dir / f'overlay_{now_ns}.png'
            if cv2.imwrite(str(path), canvas):
                self.saved += 1
                self.last_saved_ns = now_ns
                dt_ms = (now_ns - stamp_ns(scan)) / 1e6
                self.get_logger().info(
                    f'保存 {self.saved}/3：{path}，图内 {visible} 点，Δt={dt_ms:+.2f} ms'
                )


if __name__ == '__main__':
    rclpy.init()
    node = ProjectionStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'本轮共发布 {node.count} 帧投影图，保存 {node.saved} 张')
        node.destroy_node()
        rclpy.try_shutdown()
