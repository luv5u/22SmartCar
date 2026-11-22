import math
from pathlib import Path

import yaml
from geometry_msgs.msg import TransformStamped
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from sync_probe import SyncProbe, stamp_ns


def static_transform(parent, child, xyz, rpy, stamp):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.header.stamp = stamp
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
    roll, pitch, yaw = (v / 2 for v in rpy)
    cr, cp, cy = map(math.cos, (roll, pitch, yaw))
    sr, sp, sy = map(math.sin, (roll, pitch, yaw))
    q = msg.transform.rotation
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    q.w = cr*cp*cy + sr*sp*sy
    return msg


class ProjectionStream(SyncProbe):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        config_path = self.declare_parameter('extrinsics_file', 'extrinsics.yaml').value
        config = yaml.safe_load(Path(config_path).read_text())
        self.intrinsics = config['camera_intrinsics']
        stamp = self.get_clock().now().to_msg()
        transforms = [static_transform(parent, child, (0, 0, 0), (0, 0, 0), stamp)
                      for parent, child in [('map', 'odom'), ('odom', 'base_link')]]
        for child in ('laser_link', 'camera_link'):
            mount = config[child]
            transforms.append(static_transform(mount['parent'], child, mount['xyz'],
                                               mount.get('rpy', (0, 0, 0)), stamp))
        self.tf_pub = StaticTransformBroadcaster(self)
        self.tf_pub.sendTransform(transforms)
        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data
        )
        self.overlay_pub = self.create_publisher(Image, '/projection/debug_image', 5)
        self.output_dir = Path(self.declare_parameter('output_dir', '.').value)
        self.max_images = self.declare_parameter('max_images', 3).value
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.saved = 0
        self.last_saved_ns = None

    def on_info(self, msg):
        expected = [self.intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy')]
        actual = [msg.k[i] for i in (0, 4, 2, 5)]
        if not all(math.isclose(a, b, rel_tol=1e-6) for a, b in zip(actual, expected)):
            self.camera_info = None
            self.get_logger().error('bag 相机内参与 YAML 不一致', throttle_duration_sec=2.0)
            return
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
        sample = None
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
                if sample is None or abs(u - cx) < abs(sample[0] - cx):
                    sample = (u, v, distance, i)

        overlay = self.bridge.cv2_to_imgmsg(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB), encoding='rgb8')
        overlay.header = image.header
        self.overlay_pub.publish(overlay)
        self.count += 1

        # 用图像消息时间间隔选帧，回放速度改变也能取到分开的时刻。
        now_ns = stamp_ns(image)
        due = self.last_saved_ns is None or now_ns - self.last_saved_ns >= 500_000_000
        if visible and self.saved < self.max_images and due:
            path = self.output_dir / f'projection_{now_ns}.png'
            if cv2.imwrite(str(path), canvas):
                u, v, distance, beam = sample
                path.with_suffix('.txt').write_text(
                    f'pixel=({u},{v}) scan_index={beam} range_m={distance:.4f} '
                    f'scan_stamp_ns={stamp_ns(scan)}\n')
                self.saved += 1
                self.last_saved_ns = now_ns
                dt_ms = (now_ns - stamp_ns(scan)) / 1e6
                self.get_logger().info(
                    f'保存 {self.saved}/{self.max_images}：{path}，图内 {visible} 点，Δt={dt_ms:+.2f} ms'
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
