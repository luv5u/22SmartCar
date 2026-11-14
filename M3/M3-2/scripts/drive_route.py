#!/usr/bin/env python3
"""M3-2 initial route: odometry-only pure pursuit; no Gazebo truth input.
Start only from the unchanged spawn pose in the supplied scene.
Stop and restart the simulation + SLAM before retrying from the beginning.
"""
import argparse
import csv
import math
from pathlib import Path
import time


def make_route(step=0.05):
    """Coordinates relative to the initial odom pose; 12 x 8 m rounded loop.
    Assumption: car starts near world (0,-4), yaw=0 as in sim.launch.py.
    Corner radius 1 m; corridor centerlines x=+-6 and y=+-4 in world.
    """
    points = [(0.0, 0.0)]
    def line(x, y):
        ax, ay = points[-1]
        n = max(1, math.ceil(math.hypot(x-ax, y-ay)/step))
        points.extend((ax+(x-ax)*i/n, ay+(y-ay)*i/n) for i in range(1,n+1))
    def arc(cx, cy, start):
        n = math.ceil((math.pi/2)/step)
        points.extend((cx+math.cos(start+math.pi/2*i/n),
                       cy+math.sin(start+math.pi/2*i/n)) for i in range(1,n+1))
    line(5,0); arc(5,1,-math.pi/2)
    line(6,7); arc(5,7,0)
    line(-5,8); arc(-5,7,math.pi/2)
    line(-6,1); arc(-5,1,math.pi)
    line(0,0)
    return points


def tracking_step(points, index, x, y, yaw):
    # Restrict forward search: a closed path's last point equals its first.
    end = min(len(points), index+41)
    index = min(range(index, end), key=lambda i: math.hypot(points[i][0]-x, points[i][1]-y))
    error = math.hypot(points[index][0]-x, points[index][1]-y)
    target = index
    distance = 0.0
    while target < len(points)-1 and distance < 0.40:
        distance += math.hypot(points[target+1][0]-points[target][0], points[target+1][1]-points[target][1])
        target += 1
    dx, dy = points[target][0]-x, points[target][1]-y
    lateral = -math.sin(yaw)*dx+math.cos(yaw)*dy
    curvature = 2*lateral/max(dx*dx+dy*dy, 0.01)
    # Slow on turns and within 1.2 m of the narrow gate at local (0,8).
    gate = abs(x)<1.2 and y>7.0
    speed = 0.12 if abs(curvature)>0.15 or gate else 0.25
    curvature = max(-1/0.65, min(1/0.65, curvature))
    angular = speed*curvature
    done = index >= len(points)-4 and math.hypot(x,y)<0.15
    return index, error, speed, angular, done


def self_test():
    points=make_route()
    x=y=yaw=0.0; index=0; max_error=0.0
    for tick in range(20000):
        index,error,v,w,done=tracking_step(points,index,x,y,yaw)
        max_error=max(max_error,error)
        if done:
            print(f'Ideal kinematic check passed: {tick*0.05:.1f}s; max tracking error {max_error:.3f}m')
            return
        x+=v*math.cos(yaw)*0.05; y+=v*math.sin(yaw)*0.05; yaw+=w*0.05
    raise RuntimeError('Ideal kinematic route check failed to finish')


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist

    class Driver(Node):
        def __init__(self):
            super().__init__('m3_2_drive_route', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.pose=None; self.scan=None; self.odom_wall=0.; self.scan_wall=0.
            self.pub=self.create_publisher(Twist,'/cmd_vel',10)
            self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
            self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        def on_odom(self,msg):
            p=msg.pose.pose.position; q=msg.pose.pose.orientation
            self.pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            self.odom_wall=time.monotonic()
        def on_scan(self,msg):
            self.scan=msg; self.scan_wall=time.monotonic()
        def command(self,v=0.,w=0.):
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.pub.publish(msg)
        def front_clearance(self):
            scan=self.scan; valid=[]; candidates=0
            for i,r in enumerate(scan.ranges):
                angle=scan.angle_min+i*scan.angle_increment
                angle=math.atan2(math.sin(angle),math.cos(angle))
                if abs(angle)<=math.radians(20):
                    candidates+=1
                    if math.isfinite(r) and scan.range_min<=r<=scan.range_max:valid.append(r)
            # +inf can legitimately mean open space. NaN-only/empty sectors cannot.
            if not candidates:return None
            if valid:return min(valid)
            if any(math.isinf(r) and r>0 for i,r in enumerate(scan.ranges)
                   if abs(math.atan2(math.sin(scan.angle_min+i*scan.angle_increment),
                                     math.cos(scan.angle_min+i*scan.angle_increment)))<=math.radians(20)):
                return float('inf')
            return None

    rclpy.init()
    node=Driver(); output=Path(args.out).expanduser()
    stream=None
    try:
        if output.exists():raise RuntimeError(f'Output already exists: {output}; choose a fresh filename.')
        deadline=time.monotonic()+30
        while node.pose is None or node.scan is None or node.get_clock().now().nanoseconds<=0:
            if time.monotonic()>deadline:raise RuntimeError('No odom, scan or simulation clock after 30s.')
            rclpy.spin_once(node,timeout_sec=0.1)
        if node.pub.get_subscription_count()==0:raise RuntimeError('No /cmd_vel subscriber; check controller.')
        x0,y0,yaw0=node.pose
        if abs(x0)>0.3 or abs(y0)>0.3 or abs(yaw0)>0.10:
            raise RuntimeError('Odom is not near the initial spawn pose. Restart simulation and SLAM before running.')
        points=make_route(); index=0; start=node.get_clock().now().nanoseconds/1e9
        output.parent.mkdir(parents=True,exist_ok=True)
        stream=output.open('x',newline='');writer=csv.writer(stream)
        writer.writerow(['sim_time_s','odom_x_m','odom_y_m','odom_yaw_rad','local_x_m','local_y_m','local_yaw_rad','path_index','cross_track_m','cmd_v_m_s','cmd_w_rad_s','front_range_m'])
        node.get_logger().info('Starting ONE loop; Ctrl+C stops. No automatic reset or restart.')
        next_log=0.; completed=False
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0.02)
            wall=time.monotonic(); now=node.get_clock().now().nanoseconds/1e9
            if wall-node.odom_wall>3 or wall-node.scan_wall>3:raise RuntimeError('Odom/scan reception stopped for >3 wall seconds.')
            if now<start or now-start>600:raise RuntimeError('Simulation clock reset or 600s route timeout.')
            ox,oy,heading=node.pose; dx=ox-x0;dy=oy-y0
            x=math.cos(yaw0)*dx+math.sin(yaw0)*dy
            y=-math.sin(yaw0)*dx+math.cos(yaw0)*dy
            yaw=math.atan2(math.sin(heading-yaw0),math.cos(heading-yaw0))
            index,error,v,w,done=tracking_step(points,index,x,y,yaw)
            if error>0.45:raise RuntimeError(f'Path deviation {error:.2f}m exceeds 0.45m; inspect before retrying.')
            clearance=node.front_clearance()
            if clearance is None:raise RuntimeError('No usable front laser sector.')
            if clearance<0.45:raise RuntimeError(f'Front obstacle at {clearance:.2f}m; stopped for inspection.')
            if done:
                completed=True;node.command();node.get_logger().info('Loop complete; stopped. Keep SLAM running for map inspection.');break
            node.command(v,w)
            writer.writerow([now,ox,oy,heading,x,y,yaw,index,error,v,w,clearance])
            if now-start>=next_log:
                stream.flush();node.get_logger().info(f'{100*index/(len(points)-1):.0f}% | tracking error={error:.3f}m | v={v:.2f}m/s');next_log=now-start+10
        return completed
    finally:
        if rclpy.ok():
            for _ in range(10):
                node.command();rclpy.spin_once(node,timeout_sec=0.02)
        if stream is not None:stream.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',default='results/route_run_01.csv')
    parser.add_argument('--self-test',action='store_true',help='Ideal 2D kinematics only; does not validate Gazebo dynamics.')
    args=parser.parse_args()
    if args.self_test:self_test();return
    try:
        if not run(args):raise SystemExit(1)
    except KeyboardInterrupt:
        print('Interrupted; zero velocity sent if ROS remained available.')
        raise SystemExit(130)
    except Exception as error:
        print(f'STOPPED: {error}')
        raise SystemExit(1)

if __name__=='__main__':main()
