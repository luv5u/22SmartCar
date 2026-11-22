# 创意组现场展示

提前编译 M1-1、M3-1 的工作空间，每个新终端 source 对应环境。在演示机断网试跑，Gazebo 提前 10 分钟预热。提前打开原始 CSV、诊断图及关键过程录屏。

| 题目 | 现场顺序与材料 |
| --- | --- |
| M1-1（3 分钟） | 按 M1/M1-1/README.md 起 A/B/C 和报警服务端。依次执行下方命令。报警服务手动调用一次后，只停服务端，等距离再次进入报警区间，展示超时日志。最后发 status=1，指出 B 忽略异常帧的日志。 |
| M1-3（5 分钟） | 按 M1/M1-3/README.md 准备两组 bag、外参。带时钟回放、检查 TF、看三张图，切第二组重跑，再用同名 TXT 指点并报激光距离。 |
| M3-1 | 启动小车，演示接收 /cmd_vel，运行 ros2 topic hz /odom 与 ros2 topic echo /odom --once。提前录好边长 2 m 的闭环，同时记录 /odom、/gazebo/model_states。打开 logs/ten_runs_01 和 results 下的 closed_loop_stats.md、trajectory_compare.png、tf_tree.pdf。说明两条轨迹、四段 TF 的发布者与动静态；本阶段没有全局定位提供 map → odom。 |
| M3-2 | 打开 scene.world 指出 10 m 直廊、绕中央岛回环、北侧 1 m 窄门和可能的栅格收窄。现场启动 SLAM，播放建图操作录屏。按本题 README 从保存的 map.yaml/map.pgm 独立启动地图服务并激活，在 RViz 显示 /map。备好建图 bag 和质量报告中的墙线、重影、窄门数据。 |
| M3-3 | 进入 M3/M3-3，运行 ros2 launch ./launch/bringup.launch.py。确认 check_stack.py 就绪，用 RViz 的 2D Goal Pose 在地图点选 >5 m 且含转弯的目标。可用 ros2 topic info /goal_pose --verbose 检查 bt_navigator 订阅，并用 ros2 topic echo /navigate_to_pose/_action/status 观察执行和终态。录包后用 scripts/eval_run.py 导出 execution_s、global_costmap、local_costmap、cmd_vel 图。展示 results/tuning_log.md 的三组调参、三次正式尝试和失败记录。 |
| M3-4 | 按 M3/M3-4/README.md 的 0.80 m 实验说明执行。起步后 3 个仿真秒生成方块，随机位置两轮；正式对照 A/B 各 10 次，调优后 10 次。展示四指标和取舍，再现场复现一例失效。 |

M1-1 的检查终端按顺序执行（测频后 Ctrl+C）：

```bash
ros2 interface show sensor_interfaces/msg/SensorData
ros2 topic hz /processed_distance
ros2 param set /node_b_filter alpha 0.9
ros2 service call /trigger_alarm sensor_interfaces/srv/TriggerAlarm '{}'
# 停服务端，在 C 的日志看到超时后发异常帧
ros2 topic pub --once /sensor_data sensor_interfaces/msg/SensorData "{distance: 0.1, unit: m, status: 1}"
```
