# M1-3 同步与投影（5 分钟）

从本目录执行。每个终端先运行 `source /opt/ros/humble/setup.bash`。以下准备命令各运行一次，使用新的临时目录，以免替换已有 bag。把输出的绝对路径复制到其他终端的 D 变量中。

```bash
D=$(mktemp -d /tmp/m13-demo.XXXXXX)
echo "$D"
/usr/bin/python3 make_bag.py --frames 60 --seed 7 --out "$D/bag1" --extrinsics "$D/first.yaml"
/usr/bin/python3 make_bag.py --frames 60 --seed 7 --laser-z 0.45 --camera-z 0.75 --fx 420 --out "$D/bag2" --extrinsics "$D/second.yaml"
```

1. 终端一启动节点。静态 TF 由节点从指定 YAML 读取并发布；停掉以前手动启动的同名 TF 发布者。

   ```bash
   /usr/bin/python3 project_stream.py --ros-args -p use_sim_time:=true -p extrinsics_file:="$D/first.yaml" -p output_dir:="$D/images1" -p max_images:=3
   ```

2. 终端二运行 `ros2 run tf2_ros tf2_echo camera_link laser_link`，展示后 Ctrl+C。终端三可运行 `ros2 run rqt_image_view rqt_image_view /projection/debug_image`，终端二播放 `ros2 bag play "$D/bag1" --clock`。打开 images1 中三个不同时刻的 PNG。每张图的同名 TXT 给出像素坐标、激光束下标、该帧时间戳和距离（米）。
3. 停掉终端一的节点。重发第一步命令，把 first.yaml 改为 second.yaml、images1 改为 images2，再播放 bag2（带 --clock）。再次展示 TF 和第二组图片。第二组改变了雷达与相机的高度差和水平焦距。若考官提供配置和 bag，传入对应的配对文件即可。
4. 录制时钟对照时，另起节点，以 `use_sim_time:=false` 播同一个 bag，省略 `--clock` 并使用新输出目录。已有两种时钟的观测见 [验证记录](results/verification.md)。近似同步使用传感器消息头的时间戳，节点时钟按设置取仿真时间或系统时间。

静态演示中 map → odom → base_link 均为单位变换，base_link 分别连接 laser_link 和 camera_link。投影按激光时间戳查询雷达到相机的 TF，将相机前向轴作为深度；相机内参从 CameraInfo 读取并与该组 YAML 核对。图片近红远蓝，话题编码 rgb8。
