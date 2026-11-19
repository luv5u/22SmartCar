# M1-1 三节点测距、滤波与报警

## 编译

环境为 Ubuntu 22.04、ROS 2 Humble，使用系统 Python。若当前激活了 Conda 环境，先退出该环境。在 M1/M1-1 目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 运行

打开四个终端。每个终端都进入 M1/M1-1，执行 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash`，然后分别运行一条命令：

```bash
ros2 run sensor_chain node_a_sensor
ros2 run sensor_chain node_b_filter
ros2 run sensor_chain alarm_server
ros2 run sensor_chain node_c_alarm
```

A 以 10 Hz 向 `/sensor_data` 发布 `sensor_interfaces/msg/SensorData`。B 跳过 `status != 0` 的读数并打印警告，对有效距离做一阶低通滤波，将结果以 `std_msgs/msg/Float32` 发布到 `/processed_distance`。

C 在滤波距离从正常区间进入报警区间时，异步调用 `/trigger_alarm`。默认阈值 `alarm_threshold` 为 0.3 m，默认超时 `service_timeout` 为 1.0 s。报警服务端独立运行，便于单独停止并验证超时恢复。

## 验证

在已加载工作区环境的新终端执行：

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /processed_distance
```

测频后按 Ctrl+C。B 默认 `alpha=0.3`。执行以下命令，在 B 的终端观察滤波值如何随原始值变化：

```bash
ros2 param set /node_b_filter alpha 0.9
ros2 param get /node_b_filter alpha
```

手动调用报警服务，应返回 `success=True`：

```bash
ros2 service call /trigger_alarm sensor_interfaces/srv/TriggerAlarm '{}'
```

只在 `alarm_server` 的终端按 Ctrl+C。下次距离低于阈值时，C 发出请求，约 1 秒后打印“报警服务超时”。重新启动 `alarm_server`，下一次进入报警区间时，C 应打印“报警服务已确认”。
