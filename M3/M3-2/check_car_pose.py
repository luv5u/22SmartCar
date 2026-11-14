#!/usr/bin/env python3
"""Read-only diagnosis. Does not publish commands or modify SLAM/maps."""
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


def describe(label,pose):
    p,q=pose.position,pose.orientation
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    print(f'{label}: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f} m; roll={math.degrees(roll):.3f}, pitch={math.degrees(pitch):.3f}, yaw={math.degrees(yaw):.3f} deg')


def main():
    rclpy.init();node=rclpy.create_node('check_car_pose_readonly');data={}
    def on_model(msg):
        if 'smart_car' in msg.name:data['world']=msg.pose[msg.name.index('smart_car')]
        else:data['names']=msg.name
    def on_odom(msg):data['odom']=msg.pose.pose
    node.create_subscription(ModelStates,'/gazebo/model_states',on_model,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+10
        while len([key for key in ('world','odom') if key in data])<2 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
        for key in ('world','odom'):
            if key in data:describe(key,data[key])
            else:print(f'{key}: no data after 10 seconds')
        if 'names' in data:print('Model names:',data['names'])
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
