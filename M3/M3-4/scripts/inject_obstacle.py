#!/usr/bin/env python3
"""Spawn the M3-4 obstacle after the stationary car starts moving."""
import argparse
import math
from pathlib import Path
import time


def coordinates(value):
    try:
        x, y = map(float, value.split(","))
        if math.isfinite(x) and math.isfinite(y):
            return x, y
    except ValueError:
        pass
    raise argparse.ArgumentTypeError("expected two finite Gazebo world coordinates: X,Y")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--obstacle-at", type=coordinates, default=(3.0, -4.0),
                        metavar="X,Y", help="Gazebo world coordinates (default: 3,-4)")
    parser.add_argument("--delay", type=float, default=3.0,
                        help="simulation seconds after 0.05 m of displacement")
    args = parser.parse_args()
    if not math.isfinite(args.delay) or args.delay < 0:
        parser.error("--delay must be finite and nonnegative")
    model = Path(__file__).resolve().parents[1] / "models/test_box.sdf"
    if not model.is_file():
        parser.error(f"missing obstacle model: {model}")

    import rclpy
    from gazebo_msgs.srv import SpawnEntity
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node("m34_obstacle_trigger")
    try:
        client = node.create_client(SpawnEntity, "/spawn_entity")
        if not client.wait_for_service(timeout_sec=15):
            raise SystemExit("Gazebo /spawn_entity unavailable")
        request = SpawnEntity.Request()
        request.name = "m34_test_box"
        request.xml = model.read_text()
        request.reference_frame = "world"
        request.initial_pose.position.x, request.initial_pose.position.y = args.obstacle_at
        request.initial_pose.position.z = 0.4
        request.initial_pose.orientation.w = 1.0
        origin = started = future = latest = sent_at = None

        def on_odom(msg):
            nonlocal origin, started, future, latest, sent_at
            p = msg.pose.pose.position
            stamp = msg.header.stamp
            now = stamp.sec + stamp.nanosec * 1e-9
            latest = (now, p.x, p.y)
            if origin is None:
                origin = (p.x, p.y)
                print(f"READY：目标位置 {args.obstacle_at}，延时 {args.delay} 仿真秒", flush=True)
            if started is None and math.hypot(p.x-origin[0], p.y-origin[1]) >= 0.05:
                started = now
            if started is not None and future is None and now-started >= args.delay:
                sent_at = time.monotonic()
                future = client.call_async(request)
                print(f"请求生成：起步后 {now-started:.3f} 仿真秒，odom=({p.x:.3f}, {p.y:.3f})", flush=True)

        node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is None:
                continue
            if future.done():
                response = future.result()
                print(f"服务返回：success={response.success}，{response.status_message}", flush=True)
                t,x,y = latest
                print(f"返回时最近一帧里程计：起步后 {t-started:.3f} 仿真秒，odom=({x:.3f}, {y:.3f})", flush=True)
                return 0 if response.success else 1
            if time.monotonic()-sent_at > 10:
                raise SystemExit("生成请求超时，请核对现场")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
