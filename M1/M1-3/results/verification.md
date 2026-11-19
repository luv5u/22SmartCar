# M1-3 同步投影验证

## 实现

使用 ApproximateTimeSynchronizer 配对 /scan 和 /camera/image_raw，队列长度为 20，时间窗口为 50 ms。从 /camera/camera_info 读取焦距和主点，按激光消息的时间戳查询 laser_link 到 camera_link 的 TF。将视野内的激光点按距离从红到蓝着色，并将叠加图发布到 /projection/image。

## 坐标变换

静态演示中设定 map、odom、base_link 重合。以相机为参考，激光雷达原点位于 (0.05, 0, -0.10) m，两个传感器同向。camera_link 的 x 轴朝前，因此像素投影使用 x 作为深度：

u = cx - fx × y / x
v = cy - fy × z / x

雷达扫描平面比相机低 0.10 m，因此投影点位于图像水平中线下方，与首张投影图的观察一致。

## 回放结果

| 节点时钟设置 | bag 播放方式 | 发布帧数 | 保存图片 |
|---|---|---:|---:|
| use_sim_time=true | ros2 bag play sample_bag --clock | 37 | 3 |
| use_sim_time=false | ros2 bag play sample_bag | 未记录 | 3 |

两轮均成功出图。节点时钟设置改变后，消息同步仍使用传感器消息头中的时间戳。以下是 false 对照轮保存图片时的实际日志数据，Δt 定义为图像时间减去激光时间。

| 图像时间戳（s） | Δt（ms） | 图内激光点数 |
|---:|---:|---:|
| 0.093314196 | -6.71 | 66 |
| 0.684573736 | -15.69 | 66 |
| 1.203818895 | +3.30 | 66 |
