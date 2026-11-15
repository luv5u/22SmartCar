# M3-3 第一阶段：仿真与 AMCL 定位

当前仅启动 Gazebo、原有小车与控制器、地图服务器和 AMCL；尚未启用导航。
本阶段通过后再加入 Smac Hybrid + MPPI、导航行为树及实验记录。

## 放置与启动

将 config/ 和 launch/ 放到 `~/code/22SmartCar/M3/M3-3/`。
依赖已完成的 M3-2/sim.launch.py、worlds/scene.world 和 maps/map.yaml/map.pgm，
以及 M3-1 已构建的 smart_car_description。
停止旧 Gazebo、SLAM、地图服务器和运动脚本；不要同时运行两个仿真。

```bash
source /opt/ros/humble/setup.bash
source ~/code/22SmartCar/M3/M3-1/install/setup.bash
cd ~/code/22SmartCar/M3/M3-3
ros2 launch ./launch/localization.launch.py
```

首次启动期间可能短暂缺少 TF；以仿真、小车控制器均就绪后的状态判断。
不要启动 slam_toolbox、drive_route.py，也不要发布运动指令。

## 检查

在另一个同样 source 过两个环境的终端执行：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once --field pose
```

两个生命周期应为 active。静止时 AMCL 不必持续输出新位姿；晚订阅时 echo
可能等待，可先 Ctrl+C，用下面的 TF 与 RViz 检查。

```bash
ros2 run tf2_ros tf2_echo map base_footprint
rviz2 --ros-args -p use_sim_time:=true
```

RViz：Fixed Frame=map；加入 Map（/map，Durability=Transient Local）、
LaserScan（/scan，Reliability=Best Effort）和 RobotModel
（Description Topic=/robot_description）及 TF。
地图应为 M3-2 环形走廊；小车在南侧走廊中央、车头向地图 +x。
激光应与墙线大致重合。若不重合，用 2D Pose Estimate 在该位置给初值。

map -> odom 由 AMCL 根据已保存地图、激光及里程计估计；
odom -> base_footprint 由原阿克曼控制器发布；车体/雷达固定连接由
robot_state_publisher 发布。初始地图坐标 (0,0) 是近似先验，
不是读取 Gazebo 真值，也没有用静态 TF 代替定位。

## 验证范围与来源

已检查 Python 语法和 YAML 结构。生成环境没有 ROS/Gazebo，
尚未做启动或定位实测；以用户本机日志、TF 与激光叠图为准。
定位模型的参数名不表示使用差速车导航控制；规划与控制阶段仍需
匹配阿克曼最小转弯半径，禁止原地旋转恢复行为。

- https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/localization_launch.py
- https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml
- https://github.com/luv5u/22SmartCar/tree/main/M3/M3-1/src/smart_car_description
- https://github.com/luv5u/22SmartCar/tree/main/M3/M3-2

本包为阶段启动代码，不代表 M3-3 的一键全栈、三参数调优、三次实验已完成。
