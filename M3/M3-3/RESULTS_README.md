# M3-3 运行与结果

以下命令均在 M3-3 目录、已加载 ROS 2 Humble 与车辆工作空间的终端运行。保留相邻的 M3-2 仿真与地图目录。启动前结束上一轮启动的 Gazebo/Nav2。已有一次完整仿真运行时，直接使用该实例。

启动基线仿真、定位、导航和 RViz：

```bash
ros2 launch "$PWD/launch/bringup.launch.py"
```

在另一终端检查节点、雷达和 TF：

```bash
python3 scripts/check_stack.py
```

需要独立复测时，在新的终端先开启录包。每次使用一个独立目录：

```bash
ros2 bag record -o "formal_$(date +%Y%m%d_%H%M%S)" --include-hidden-topics /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback /plan /odom /tf /tf_static /clock /scan /cmd_vel /cmd_vel_nav /local_costmap/costmap /global_costmap/costmap /amcl_pose
```

在另一终端发目标：

```bash
python3 scripts/goto.py --x 6 --y 4 --yaw 1.57079633 --runs 1
```

结果追加到 results/goto.csv。多次模式可用 --runs 3，每次之间按提示结束上一录包、复位仿真与定位并新开录包。应答超时后先核实服务端状态。客户端测量从接受目标到收到结果的墙钟时间，录包分析使用状态消息的接收时间。

目标结束后，在录包终端按 Ctrl+C 并等待 Recording stopped。在 M3-3 中运行分析，将示例路径换成录包目录或其单个 db3 文件：

```bash
python3 scripts/eval_run.py /path/to/bag --out results
```

分析依赖 Python 3、NumPy、Matplotlib，输出同名 CSV 和 PNG。图中依次为首条规划与定位估计轨迹、最终速度指令、执行区间中间一帧局部代价地图。无执行状态的窗口留空。脚本针对每包单目标、单 SQLite 分片。配置文件 nav2_speed025.yaml、nav2_infl040.yaml、nav2_progress010.yaml 分别对应单变量实验。启动时可传入 params_file 的绝对路径。

要使用 ROS 包名入口，可先在 M3-3 目录构建：

```bash
colcon build --base-paths . --packages-select m3_navigation
```

```bash
source install/setup.bash
```

```bash
ros2 launch m3_navigation bringup.launch.py
```

包名入口指向构建时的源码位置，移动仓库后需重新构建。结果表见 results/tuning_log.md，过弯轨迹叠图见 results/inflation_comparison.png。项目保留原启动入口。
