#!/usr/bin/env python3
"""M3-1：按时间走圆角正方形；独立重复、复位、采集双来源轨迹。

注意：复位真值仅用于两次实验之间重新摆车，不修正里程计。
CSV 保留各来源原始坐标；起终点差值及局部坐标对比另存 summary.csv。
先用 --laps 1 验证，再跑 --laps 10。必须在已激活的 ROS2 环境运行。
"""

import argparse
import csv
import math
from pathlib import Path
import statistics
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def wrap(angle):
    """归一化到 (-pi, pi]。"""
    result = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if result <= -math.pi else result


def pose_xyyaw(pose):
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (pose.position.x, pose.position.y, wrap(yaw))


def displacement(start, end):
    """把位移转到该来源本圈起始朝向；不把真值反馈给里程计。"""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap(end[2] - start[2]))


def make_stages(side, radius, speed):
    straight_time = (side - 2.0 * radius) / speed
    turn_rate = speed / radius
    turn_time = math.pi / (2.0 * turn_rate)
    stages = []
    for index in range(4):
        stages.append((f'第{index + 1}段直行', straight_time, speed, 0.0))
        stages.append((f'第{index + 1}次左转', turn_time, speed, turn_rate))
    return stages


class SquareExperiment(Node):
    def __init__(self, args):
        super().__init__('square_experiment', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('本实验必须使用仿真时间，请勿关闭 use_sim_time')
        self.args = args
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.truth_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.on_truth,
            qos_profile_sensor_data)
        self.samples = {}
        self.stages = make_stages(args.side, args.radius, args.speed)
        self.duration = sum(stage[1] for stage in self.stages)
        self.state = 'WAIT'
        self.state_wall = time.monotonic()
        self.last_clock = None
        self.clock_wall = self.state_wall
        self.lap = 0
        self.recording = False
        self.files = {}
        self.writers = {}
        self.rows = []
        self.done = False
        self.failed = False
        self.future = None
        self.last_stage = -1
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'计划 {args.laps} 次，单次运动指令约 {self.duration:.2f} 秒；'
            f'输出：{args.out}。等待时钟、控制器及两路轨迹。')

    def clock_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def capture(self, source, pose, twist):
        t = self.clock_seconds()
        point = pose_xyyaw(pose)
        moving = max(abs(twist.linear.x), abs(twist.linear.y),
                     abs(twist.angular.z))
        self.samples[source] = (t, point, time.monotonic(), moving)
        if self.recording:
            self.write_point(source, t, point)

    def on_odom(self, msg):
        self.capture('odom', msg.pose.pose, msg.twist.twist)

    def on_truth(self, msg):
        if self.args.model not in msg.name:
            return
        i = msg.name.index(self.args.model)
        self.capture('truth', msg.pose[i], msg.twist[i])

    def write_point(self, source, t, point):
        self.writers[source].writerow(
            [f'{t:.9f}', *(f'{value:.9f}' for value in point)])

    def fresh(self):
        wall = time.monotonic()
        return all(source in self.samples
                   and wall - self.samples[source][2] < 2.0
                   for source in ('odom', 'truth'))

    def stopped(self):
        return all(self.samples[source][3] < 0.01
                   for source in ('odom', 'truth'))

    def zero(self):
        self.publisher.publish(Twist())

    def change(self, state, now):
        self.state = state
        self.state_time = now
        self.state_wall = time.monotonic()

    def request_reset(self, now):
        self.zero()
        self.lap += 1
        request = SetEntityState.Request()
        request.state.name = self.args.model
        request.state.reference_frame = 'world'
        request.state.pose.position.z = self.args.reset_z
        request.state.pose.orientation.w = 1.0
        self.future = self.reset_client.call_async(request)
        self.change('RESET', now)
        self.get_logger().info(f'准备第 {self.lap}/{self.args.laps} 次：复位小车')

    def begin_run(self, now):
        self.starts = {key: self.samples[key][1] for key in ('odom', 'truth')}
        for source in ('odom', 'truth'):
            path = Path(self.args.out) / f'run_{self.lap:02d}_{source}.csv'
            self.files[source] = path.open('x', newline='', encoding='utf-8')
            self.writers[source] = csv.writer(self.files[source])
            self.writers[source].writerow(['t', 'x', 'y', 'theta'])
            self.write_point(source, self.samples[source][0], self.starts[source])
        self.recording = True
        self.last_stage = -1
        self.change('DRIVE', now)

    def close_recording(self):
        self.recording = False
        for handle in self.files.values():
            handle.close()
        self.files.clear()
        self.writers.clear()

    def finish_run(self, now):
        values = {}
        row = {'run': self.lap}
        for source in ('odom', 'truth'):
            t, end, _, _ = self.samples[source]
            self.write_point(source, t, end)
            delta = displacement(self.starts[source], end)
            values[source] = delta
            row[f'{source}_position_m'] = math.hypot(delta[0], delta[1])
            row[f'{source}_angle_deg'] = abs(math.degrees(delta[2]))
        # 两来源分别减起点并转至本圈初始朝向，再比较增量。
        odom, truth = values['odom'], values['truth']
        row['odom_truth_delta_error_m'] = math.hypot(
            odom[0] - truth[0], odom[1] - truth[1])
        row['odom_truth_yaw_error_deg'] = abs(math.degrees(
            wrap(odom[2] - truth[2])))
        self.close_recording()
        self.rows.append(row)
        with (Path(self.args.out) / 'summary.csv').open(
                'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(
            f'第 {self.lap} 次已保存：里程计闭环偏差 '
            f'{row["odom_position_m"]:.4f} m，'
            f'真值闭环偏差 {row["truth_position_m"]:.4f} m。')
        if self.lap < self.args.laps:
            self.request_reset(now)
        else:
            for key in list(row)[1:]:
                data = [item[key] for item in self.rows]
                sd = statistics.stdev(data) if len(data) > 1 else None
                text = f'{statistics.mean(data):.6f}'
                text += f' ± {sd:.6f}' if sd is not None else '（仅一次，未计算标准差）'
                self.get_logger().info(f'{key}: {text}')
            self.get_logger().info('全部完成。文件已关闭；未宣称误差合格，下一步检查轨迹。')
            self.done = True

    def tick(self):
        now = self.clock_seconds()
        if self.done:
            self.zero()
            return
        if self.state == 'WAIT':
            self.zero()
            if (now > 0.0 and self.fresh()
                    and self.reset_client.service_is_ready()
                    and self.publisher.get_subscription_count() > 0
                    and time.monotonic() - self.state_wall > 2.0):
                if self.count_publishers('/cmd_vel') > 1:
                    raise RuntimeError('还有其他 /cmd_vel 发布者，请先停止旧运动节点')
                self.request_reset(now)
            return
        if not self.fresh():
            raise RuntimeError('轨迹数据超过两秒未更新；已停止实验，请检查仿真')
        if self.state == 'RESET':
            self.zero()
            if self.future.done():
                response = self.future.result()
                if response is None or not response.success:
                    raise RuntimeError('小车复位服务返回失败')
                self.change('SETTLE', now)
            elif time.monotonic() - self.state_wall > 15.0:
                raise RuntimeError('等待复位服务超时')
        elif self.state == 'SETTLE':
            self.zero()
            if (now - self.state_time >= 1.0 and self.stopped()
                    and all(self.samples[k][2] > self.state_wall
                            for k in ('odom', 'truth'))):
                truth = self.samples['truth'][1]
                if math.hypot(truth[0], truth[1]) > 0.05 or abs(truth[2]) > 0.1:
                    raise RuntimeError('复位后位置或朝向未接近目标，请检查模型复位')
                self.begin_run(now)
            elif now - self.state_time > 8.0:
                raise RuntimeError('复位后小车未停稳，未开始记录')
        elif self.state == 'DRIVE':
            elapsed = now - self.state_time
            end_time = 0.0
            msg = Twist()
            for index, (name, duration, speed, rate) in enumerate(self.stages):
                end_time += duration
                if elapsed < end_time:
                    msg.linear.x, msg.angular.z = speed, rate
                    if self.last_stage != index:
                        self.get_logger().info(f'第 {self.lap} 次：{name}')
                        self.last_stage = index
                    break
            else:
                self.change('STOP', now)
            self.publisher.publish(msg)
        elif self.state == 'STOP':
            self.zero()
            if now - self.state_time >= 1.0 and self.stopped():
                self.finish_run(now)
            elif now - self.state_time > 8.0:
                raise RuntimeError('运动结束后未停稳，本次不写为成功结果')

    def watchdog(self):
        """壁钟超时检查：仿真暂停时，仿真定时器本身也会暂停。"""
        now = self.clock_seconds()
        wall = time.monotonic()
        if self.last_clock is not None and now < self.last_clock - 1e-6:
            raise RuntimeError('仿真时钟倒退；请勿在实验中重置仿真')
        if self.last_clock is None or now != self.last_clock:
            self.clock_wall = wall
        self.last_clock = now
        if wall - self.clock_wall > 20.0:
            raise RuntimeError('仿真时钟超过20秒未推进，请确认仿真正在播放')
        if self.state == 'WAIT' and wall - self.state_wall > 30.0:
            raise RuntimeError('等待时钟、两路轨迹或复位服务超时')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--side', type=float, default=2.0)
    parser.add_argument('--radius', type=float, default=0.6)
    parser.add_argument('--speed', type=float, default=0.2)
    parser.add_argument('--out', default='logs/check_01')
    parser.add_argument('--model', default='smart_car')
    parser.add_argument('--reset-z', type=float, default=0.06)
    args, ros_args = parser.parse_known_args()
    min_radius = 0.28 / math.tan(math.pi / 6.0) + 0.18 / 2.0
    if not (args.laps > 0 and all(math.isfinite(v) for v in (
            args.side, args.radius, args.speed, args.reset_z))
            and 0 < args.speed <= 1.0
            and min_radius <= args.radius < args.side / 2.0
            and args.reset_z > 0):
        parser.error(f'圈数需为正；0<速度<=1；{min_radius:.3f}<=半径<边长/2；高度>0')
    out = Path(args.out)
    if out.exists() and any(out.iterdir()):
        parser.error('输出目录非空，请换一个目录名，避免覆盖旧数据')
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = None
    exit_code = 0
    try:
        node = SquareExperiment(args)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.watchdog()
    except KeyboardInterrupt:
        print('手动中断：保留已有数据，最后一次可能不完整，请勿计入正式统计。')
        exit_code = 130
    except Exception as error:
        print(f'实验失败：{error}。未完成的最后一次CSV仅供排查。')
        exit_code = 1
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(5):
                    node.zero()
                    time.sleep(0.02)
            node.close_recording()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
