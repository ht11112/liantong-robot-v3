# RK ROS Noetic 接入

`catkin_ws` 是独立的 ROS1 工作空间。使用 RK 的 ROS Noetic 和 Python 3.8，在仓库根目录执行构建命令。

```bash
source /opt/ros/noetic/setup.bash
cd rk/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 仅定位

先确认雷达驱动正在发布 `/lidar1/laser_fan`，准备匹配现场的地图 YAML 及图像，核对 V3 底盘 RS485 的设备路径和串口归属。

```bash
roslaunch liantong_v3 localization.launch port:=/dev/serial/by-path/实际设备 map_yaml:=/绝对路径/map.yaml
```

默认只发送 `QUERY`。`/odom` 和 `odom → base_link` 来自同一条新鲜实测样本，使用相同测量时间戳。激光节点发布 `base_link → laser` 静态变换及 `/scan`。`map_server` 提供保存地图，持续运行的 AMCL 是 `map → odom` 的唯一发布者。用 RViz 的「2D Pose Estimate」或 `/initialpose` 输入实测地图位置与朝向，底盘 odom 原点保持连续。RK 有桌面时可附加 `start_rviz:=true`。

激光默认前移 0.13 m、侧移 0、离地 0.20 m、yaw 0 rad。用 `laser_x`、`laser_y`、`laser_z`、`laser_yaw` 传实测值。导航只使用车头前 180°。`/scan` 的时间戳是首个保留射线时刻，保留原 `time_increment`。`min_usable_range` 默认 1 m，低于下限及无效回波标为 NaN。代价地图 `inf_is_valid` 为 `false`，真实无回波也不作为清空射线。前方近场和后方仍有探测盲区，原雷达话题保持原始数据。

桥接器以 50 ms 为 QUERY 目标周期。每次只进行一个请求，100 ms 内未收到完整回复、回复格式异常或串口出错时退出，`roslaunch` 随必需节点退出。首次有效 ODO 以查询发起时刻减去 MCU 报告的源龄建立单调时钟锚点，之后用 MCU 实测 tick 差推进。重复和无效样本不重新发布。已经发布后如 MCU boot 或 odom segment 改变、时序倒退或时钟映射失效，当前定位会话结束，重新启动后需重新输入地图初始位姿。

## 可选导航与控制

`navigation.launch` 加载同一套定位和 `move_base`，其 `/cmd_vel` 可在关闭运动时用于观察规划。实际转发需要明确传入 `enable_motion:=true`。随附 DWA 直行指令上限为 0.03 m/s，角速度采样下限 0.07 rad/s、上限 0.14 rad/s。按名义轮径和轮距，纯转约对应每轮 1 到 2 RPM，避免低速纯转给定量化为零。参数是指令限制，实物角速度以标定结果为准。用户直接发布的 `/cmd_vel` 不经过 DWA，线协议与 MCU 按每分量 ±1 限幅。

```bash
roslaunch liantong_v3 navigation.launch port:=/dev/serial/by-path/实际设备 map_yaml:=/绝对路径/map.yaml
```

启用运动时才提供 `/liantong_v3_bridge/stop`、`/liantong_v3_bridge/resume` 两个 `std_srvs/Trigger` 服务。服务成功表示 MCU 回了对应的解析 ACK。STOP 清当前速度并锁存，锁存期间桥只查询和处理显式控制请求。RESUME 解除锁存，桥只使用在收到 RESUME ACK 后才进入回调的新速度消息。`Twist` 没有源时间戳，默认 0.3 秒有效期从 RK 单调时钟的回调接收时刻起算，过期停止发送 CMD 但继续 QUERY。MCU 的 510 ms 无速度命令停车从最后一条收到的有效 CMD 起算。

## 本地离线检查

从仓库根目录运行。测试使用本机 socketpair 验证实际串口事务读取逻辑的字节处理和超时，并验证协议、时戳和扫描裁剪。

```bash
PYTHONPATH=rk/catkin_ws/src/liantong_v3/src python3 -B -m unittest discover -s rk/catkin_ws/src/liantong_v3/tests
```
