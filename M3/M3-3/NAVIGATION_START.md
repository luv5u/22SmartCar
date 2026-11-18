# M3-3 第二阶段：启动自主导航全栈

日期：2026-11-17。当前提供初始基线，不是已经测出的最终调优结果。
第一阶段已在用户本机确认 AMCL/map_server active、map -> base_footprint
接通、静止时激光与地图大致对齐。运动中定位、导航成功率仍待实测。

## 文件与放置

将本包直接解压到 `~/code/22SmartCar/M3/M3-3/`，保留已有文件。
本包仅新增第二阶段文件，不替换原有 amcl_params.yaml、localization.launch.py
或 localization.rviz。启动继续复用已验证的第一阶段配置及 M3-2 仿真、地图。
不需要重新建图，也不需要为这些直接路径启动的文件运行 colcon。

新增内容：

- config/nav2_params.yaml：Humble 的 Smac Hybrid + Ackermann MPPI 配置。
- launch/bringup.launch.py：单命令启动仿真、地图、AMCL、Nav2、RViz。
- behavior_trees/*.xml：清除代价地图与等待恢复，移除原地旋转、倒车。
- navigation.rviz：复用已有视角与激光显示，增加两张代价地图、全局路径。
- scripts/check_stack.py：只读状态检查，约 15 秒内结束，不发目标或速度。
- results/tuning_log.md：区分初始适配与后续实测对比，未编造实验结果。

## 启动

先在旧的启动终端 Ctrl+C 停止 localization.launch.py 或其他仿真，
并关闭旧 RViz。不要同时运行 slam_toolbox、drive_route.py、旧地图服务器
或遥控程序，以免重复发布 TF/速度。

依次执行，每个代码框是一条命令：

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/code/22SmartCar/M3/M3-1/install/setup.bash
```

```bash
cd ~/code/22SmartCar/M3/M3-3
```

```bash
ros2 launch ./launch/bringup.launch.py
```

此命令自动打开 Gazebo 和 RViz。没有发送导航目标时，小车应保持静止。
启动阶段可能出现暂时缺少 TF 的提示；等小车和控制器启动后再判断。
若持续报错，不发布目标，保留完整终端输出供排查。

要关闭自动 RViz，可使用另一种启动方式（不是同时再启动一次）：

```bash
ros2 launch ./launch/bringup.launch.py rviz:=false
```

## 检查

新开终端，逐条执行：

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/code/22SmartCar/M3/M3-1/install/setup.bash
```

```bash
cd ~/code/22SmartCar/M3/M3-3
```

```bash
python3 scripts/check_stack.py
```

九个生命周期节点应均为 active，并打印雷达 frame、最近消息年龄、
map -> base_footprint 和三个包的实际版本。末行 READY 只表示服务、激光、
TF 基础就绪，不证明导航成功。启动未完全结束时可等就绪后再运行一次。
RViz 中代价地图和激光应出现在原图上；还没发目标时 Global Plan 为空正常。
如果画面被两层代价地图遮挡，可临时取消其中一项勾选。

## 当前配置的依据

| 环节 | 基线 | 原因 |
| --- | --- | --- |
| 定位 | 原有 AMCL 配置 | 地图与激光匹配；不使用 Gazebo 真值，不额外发静态 map -> odom |
| 全局规划 | SmacPlannerHybrid / DUBIN | 向前行驶，搜索包含转弯约束，避免差速车默认栅格路径 |
| 控制 | MPPI / Ackermann | 直接约束 v / omega，零前进速度下不能原地转向 |
| 转弯半径 | 规划、控制均为 0.65 m | 依据轴距 0.28 m、轮距 0.18 m、单前轮转角限位约 30°，留保守余量 |
| 碰撞轮廓 | 长 0.50 m、宽 0.30 m 的矩形，另加 1 cm padding | 底盘 0.45 × 0.22 m，包含相机前伸、车轮和少量几何余量 |
| 速度 | 前进不超过 0.35 m/s，不倒车 | 初始低速运行，便于观察窄门、转角与定位 |
| 恢复 | 清图、等待；有限重试 | 不向阿克曼小车要求原地旋转或差速式倒车恢复 |

近似几何校核：限制的是内侧前轮转角，后轴中点最小半径约为
0.28/tan(30°)+0.18/2 = 0.575 m；换算至车体中心约 0.592 m。
0.65 m 是初始保守配置，仍需通过运动实测验证，而不是已测出的转弯半径。
规划器与 MPPI 使用简化的平面模型，不保证底层车辆无滑移。

MPPI 的 56 × 0.05 = 2.8 s 预测时域，以 0.35 m/s 预测约 0.98 m，
小于局部代价地图 4 m 边长的一半（2 m），并留有车身余量。
速度平滑器在 MPPI 后限制加减速；后续必须记录最终 /cmd_vel，
检查平滑后的曲率与实际控制表现，不能只看规划器参数就声称约束全部满足。

窄门：M3-2 实测自由宽度约 0.95 m；加 padding 后矩形宽度为 0.32 m。
正对门时名义横向总余量约 0.63 m，但倾斜进门会增加投影宽度。
inflation_radius=0.55 m 主要定义代价扩展范围，**不等于整个范围均禁止进入**。
两侧代价重叠不能单独证明门被堵死；还要检查硬碰撞区、姿态、规划和实跑。

速度链：controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
-> 原 ackermann_steering_controller。不得同时运行其他速度发布程序。
TF 链：AMCL 发布 map -> odom；原车控制器发布 odom -> base_footprint；
robot_state_publisher 发布车体/传感器链。AMCL 的 DifferentialMotionModel
是非完整平面里程计误差模型，不是用于驱动车轮的差速控制器。

## 下一步实验（尚未完成）

1. 本机确认全栈激活及代价地图正常，再选择含转弯、规划长度 >5 m 的目标。
2. 加入发目标、保存规划/实际估计轨迹、cmd_vel 与录包的实验记录。
3. 在相同起点/目标条件下逐项改变至少三个参数，保留改前/改后数据和失败。
4. 每次恢复相同初始条件后做三次正式试验；不能在终点重复发送目标凑三次。
5. 完成两张代价地图、规划和实际轨迹、速度曲线及最终 README。

当前没有预设或自动发布目标。RViz 普通 2D Goal Pose 是否与 Humble Nav2
动作客户端衔接需另行确认，后续优先使用明确调用 NavigateToPose 的记录脚本。

## 已做的检查与限制

生成环境没有 ROS/Gazebo，未声称已在本地启动导航或完成实跑。
已静态核对 Python 语法、YAML 重复键与关键类型、两套转弯半径、速度约束、
局部地图/预测长度、BT 的恢复节点及其 Humble 插件名。实际版本由检查脚本打印。
没有实测成功率、耗时、碰撞率或调参提升数字。

## 来源

- [用户仓库](https://github.com/luv5u/22SmartCar/tree/main/M3)
- [M3-3 原题](https://github.com/Blake-John/22SmartCar/blob/main/round1/M3/M3-3/README.md)
- [Humble Nav2 默认参数](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
- [Humble 导航启动源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py)
- [Smac 参数与几何说明](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/README.md)
- [MPPI 参数](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md)
- [MPPI Ackermann 约束源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp)

两份行为树改编自 Navigation2 Humble（Apache-2.0），原始版权与授权说明
见原项目及本包 THIRD_PARTY_NOTICE.md。
