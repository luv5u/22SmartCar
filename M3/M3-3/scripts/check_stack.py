#!/usr/bin/env python3
"""Read-only, bounded readiness check. Does not publish goals or velocities."""
import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    node = Node('m3_3_stack_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {}
    subscription = node.create_subscription(
        LaserScan, '/scan', lambda msg: latest.update(scan=msg), qos_profile_sensor_data)
    names = ['map_server', 'amcl', 'planner_server', 'controller_server',
             'bt_navigator', 'behavior_server', 'smoother_server',
             'velocity_smoother', 'waypoint_follower']
    clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in names}
    futures = {}
    deadline = time.monotonic() + 15.0
    # Services are discovered concurrently, avoiding 9 sequential timeouts.
    while time.monotonic() < deadline:
        for name, client in clients.items():
            if name not in futures and client.service_is_ready():
                futures[name] = client.call_async(GetState.Request())
        rclpy.spin_once(node, timeout_sec=0.1)
        ready_tf = buffer.can_transform('map', 'base_footprint', rclpy.time.Time())
        if (len(futures) == len(names) and all(f.done() for f in futures.values())
                and 'scan' in latest and ready_tf):
            break
    ok = True
    for name in names:
        future = futures.get(name)
        result = future.result() if future and future.done() and not future.exception() else None
        active = bool(result and result.current_state.id == State.PRIMARY_STATE_ACTIVE)
        label = result.current_state.label if result else 'no response'
        print(f'{name}: {label}')
        ok &= active
    scan = latest.get('scan')
    fresh = False
    if scan is not None:
        age = (node.get_clock().now().nanoseconds -
               (scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec)) / 1e9
        fresh = -0.1 <= age <= 2.0
        print(f'/scan: frame={scan.header.frame_id}, beams={len(scan.ranges)}, age={age:.2f}s')
    else:
        print('/scan: NO MESSAGE')
    ok &= fresh
    try:
        transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        print(f'map -> base_footprint: x={t.x:.3f}, y={t.y:.3f}, yaw={math.degrees(yaw):.2f}deg')
    except TransformException as error:
        print(f'map -> base_footprint: MISSING ({error})')
        ok = False
    for package in ['nav2_bringup', 'nav2_smac_planner', 'nav2_mppi_controller']:
        try:
            path = get_package_share_directory(package) + '/package.xml'
            version = ET.parse(path).getroot().findtext('version')
            print(f'{package}: {version}')
        except Exception as error:
            print(f'{package}: {error}')
            ok = False
    print('READY: initial services/scan/TF only; navigation still untested.' if ok
          else 'NOT READY: inspect inactive nodes and launch errors above.')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
