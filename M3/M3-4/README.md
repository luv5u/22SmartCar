# M3-4 动态障碍避让

本目录依赖同一仓库中已跑通的 M3-1 小车模型、M3-2 地图及 M3-3 导航栈。分析结论和实验范围见 [四问报告](report.md)，每轮数据见 [benchmark.csv](results/benchmark.csv)。

## 环境准备

在原 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 环境中执行。先结束之前的手动 launch 和 rosbag 录制。批量脚本每轮自行启动并关闭仿真。使用同一台机器的 Python 3、ROS 客户端及 M3-3 的 check_stack.py 和 goto.py。

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/code/22SmartCar/M3/M3-1/install/local_setup.bash
```

```bash
cd ~/code/22SmartCar/M3/M3-4
```

## 一轮验收演示

```bash
/usr/bin/python3 scripts/run_benchmark.py --planners A --runs 1 --obstacle-at=3,-4 --delay 3 --out results/demo_runs
```

`--obstacle-at` 使用 Gazebo 世界坐标，车的出生点为 (0, −4)，导航目标使用地图坐标 (6, 4)。随机障碍演示将坐标换成考官给的值，把次数改为 2。触发器以驶离初始位置 0.05 m 为计时起点，延时采用里程计仿真时间。接近失效条件可尝试 (2, −4)。

## 批量复现

```bash
/usr/bin/python3 scripts/run_benchmark.py --planners A,B --runs 10 --obstacle-at=3,-4 --delay 3 --out results/reproduced
```

```bash
/usr/bin/python3 scripts/run_benchmark.py --planners C --runs 10 --obstacle-at=3,-4 --delay 3 --out results/reproduced
```

每次创建带时间戳的新目录。每轮重启仿真，检查初始定位与权重，开启录包并等待触发器就绪，然后发送目标。每个目录保存 launch、录包、触发、目标与就绪日志、权重和目标 CSV。会话根目录的 benchmark.csv 逐行记录结果。spawn_ok 指示是否有效注入；到达且有效注入的轮次参与成功耗时和绕行统计。若录包损坏或启动失败，脚本保存已知状态和错误，结束本批。修复后可以从新会话补足剩余次数。额外路径默认使用本路线已实测的无障碍基线 8.7051282 m，更换路线时用 --baseline-m 指定同口径基线。

## 核对已有实验

正式实验由三个终端逐轮执行，记录覆盖 30 次有效试验、2 次注入异常及 C10 的额外请求。批量入口用于后续复现。报告沿用原配置名 config/mppi_a.yaml、mppi_b.yaml、mppi_c.yaml。轨迹图基于 odom，方块按初始地图与世界坐标关系标出。方块边长 0.40 m，小于题目建议的 0.5～0.8 m；正式结果仅对应实际尺寸。

从已导出的轻量数据包再次计算原实验指标和绘图：

```bash
/usr/bin/python3 scripts/eval_metrics.py ~/M3-4_analysis_input.zip --out /tmp/m34_metrics_check
```

此步骤使用 matplotlib。逐轮指标、失败调试日志分析和调优比较分别见 results/benchmark.csv、results/failure_boundary.md 和 results/tuning_log.md。
