"""Send independent NavigateToPose trials and append one CSV row per attempt."""
import argparse
import csv
import math
from pathlib import Path
from time import monotonic

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


def wait(node, future, seconds):
    deadline = monotonic() + seconds
    while rclpy.ok() and not future.done() and monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.result() if future.done() else None


def run(node, client, args):
    if not client.wait_for_server(timeout_sec=10):
        return 'SERVER_UNAVAILABLE', None, ''
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2)
    handle = wait(node, client.send_goal_async(goal), args.ack_timeout)
    if handle is None:
        return 'GOAL_RESPONSE_TIMEOUT', None, ''
    if not handle.accepted:
        return 'REJECTED', None, ''
    goal_id = bytes(handle.goal_id.uuid).hex()
    start = monotonic()
    result = wait(node, handle.get_result_async(), args.timeout)
    if result is None:
        wait(node, handle.cancel_goal_async(), 5)
        return 'RESULT_TIMEOUT', None, goal_id
    return {4:'SUCCEEDED', 5:'CANCELED', 6:'ABORTED'}.get(result.status,'UNKNOWN'), monotonic()-start, goal_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x',type=float,default=6.0)
    parser.add_argument('--y',type=float,default=4.0)
    parser.add_argument('--yaw',type=float,default=math.pi/2,help='Radians in map')
    parser.add_argument('--runs',type=int,default=1)
    parser.add_argument('--ack-timeout',type=float,default=15)
    parser.add_argument('--timeout',type=float,default=120)
    parser.add_argument('--out','--save',type=Path,default=Path('results/goto.csv'))
    args = parser.parse_args()
    if min(args.runs,args.timeout,args.ack_timeout) <= 0:
        parser.error('Runs and timeouts must be positive')
    args.out.parent.mkdir(exist_ok=True,parents=True)
    rclpy.init()
    node = Node('m3_goal_runner',parameter_overrides=[Parameter('use_sim_time',value=True)])
    client = ActionClient(node,NavigateToPose,'/navigate_to_pose')
    try:
        with args.out.open('a', newline='') as f:
            writer = csv.writer(f)
            if f.tell() == 0:
                writer.writerow(['run','x','y','yaw_rad','outcome','accept_to_result_wall_s','goal_id'])
            for n in range(1,args.runs+1):
                if n > 1:
                    input('请重置仿真与定位到同一起点、确认导航就绪并开启新录包，再按回车：')
                status,seconds,goal_id = run(node,client,args)
                writer.writerow([n,args.x,args.y,args.yaw,status,seconds,goal_id])
                f.flush()
                print(f'Run {n}: {status}',flush=True)
                if status in ('RESULT_TIMEOUT','GOAL_RESPONSE_TIMEOUT'):
                    print('服务端目标状态待核实，本批停止。')
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
