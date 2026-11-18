from math import hypot
from pathlib import Path
from time import monotonic

import rclpy
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("m34_obstacle_trigger")
client = node.create_client(SpawnEntity, "/spawn_entity")
request = SpawnEntity.Request()
request.name = "m34_test_box"
request.xml = (Path(__file__).resolve().parents[1] / "models" / "test_box.sdf").read_text()
request.reference_frame = "world"
request.initial_pose.position.x = 3.0
request.initial_pose.position.y = -4.0
request.initial_pose.position.z = 0.4
request.initial_pose.orientation.w = 1.0

origin = None
started = None
future = None
latest = None
sent_at = None

def on_odom(msg):
    global origin, started, future, latest, sent_at
    p = msg.pose.pose.position
    stamp = msg.header.stamp
    now = stamp.sec + stamp.nanosec * 1e-9
    latest = (now, p.x, p.y)
    if origin is None:
        origin = (p.x, p.y)
        print("READY：服务和里程计已就绪，可以发导航目标", flush=True)
    if started is None and hypot(p.x - origin[0], p.y - origin[1]) >= 0.05:
        started = now
    if started is not None and future is None and now - started >= 3.0:
        future = client.call_async(request)
        sent_at = monotonic()
        print(f"请求生成：起步后 {now-started:.3f} 仿真秒，"
              f"odom=({p.x:.3f}, {p.y:.3f})", flush=True)

try:
    if not client.wait_for_service(timeout_sec=15):
        raise SystemExit("未找到 /spawn_entity，请检查 Gazebo 是否已启动")
    node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if future is None:
            continue
        if future.done():
            response = future.result()
            t, x, y = latest
            print(f"服务返回：success={response.success}，{response.status_message}")
            print(f"返回时最近一帧里程计：起步后 {t-started:.3f} 仿真秒，"
                  f"odom=({x:.3f}, {y:.3f})")
            break
        if monotonic() - sent_at > 10:
            raise SystemExit("生成请求已发出，10 秒内未收到回复。请先检查现场。")
finally:
    node.destroy_node()
    rclpy.shutdown()
