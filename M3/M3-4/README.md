> 2026-11-22 更新：当前正式报告采用 0.80 m 方块。已完成 A/B 各 10 次有效注入、C 调优后 10 次有效注入。新数据和图片见 [results/box080](results/box080/)，结论见 [四问报告](report.md)。下文“核对已有实验”介绍较早的 0.40 m 历史数据。

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

## 0.80 m 方块展示

上文报告、历史轨迹图和历史 30 次有效实验均对应 0.40 m 方块。当前 test_box.sdf 的碰撞与可视尺寸均为 0.80 × 0.80 × 0.80 m。新脚本会把实际模型存入批次目录，CSV 记录尺寸、世界坐标、触发延时；轮次目录中的 trajectory.csv 保存按目标执行窗口截取的里程计轨迹。

```bash
/usr/bin/python3 scripts/run_benchmark.py --planners A --runs 1 --obstacle-at=3,-4 --delay 3 --out results/box080_pilot
/usr/bin/python3 scripts/run_benchmark.py --planners A,B --runs 10 --obstacle-at=3,-4 --delay 3 --out results/box080_formal
```

先看试跑的 trigger.log、goal.log 和实车仿真画面。保存接近、检测、绕行、通过四帧。正式 A/B 各取 10 次有效注入；保留中止、注入失败的原始记录。比较指标后，说明对较差方案的调优理由。若采用已有的中间权重 5.715，再执行：

```bash
/usr/bin/python3 scripts/run_benchmark.py --planners C --runs 10 --obstacle-at=3,-4 --delay 3 --out results/box080_tuned
```

考官指定位置时，替换 --obstacle-at 的世界坐标，使用 --runs 2 和单独的输出目录。另选靠近起点的位置单独验证失效，记录实际表现和日志。旧报告的失效边界和成功率仅用于原尺寸。根据新批次重新生成图表、填写四问报告，并记录未产生新路径等观察限制。

```bash
python3 scripts/plot_batches.py results/box080_formal/batch_* results/box080_tuned/batch_* --out results/box080_summary
```

只传同一实验条件的正式批次。汇总表保留每组有效注入次数和成功次数，可用于检查是否需要补测。
