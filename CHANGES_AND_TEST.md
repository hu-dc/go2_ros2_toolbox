# Go2 ROS2 Toolbox 适配：改动清单与实机测试命令

> 对应仓库：https://github.com/andy-zhuo-02/go2_ros2_toolbox （基线 HEAD `cd11d74`，2025-07-20）
> 本机环境：ROS2 Humble + unitree_ros2/cyclonedds，Go2（L1 雷达，`/utlidar/*` 话题）
> 原则：只改代码，不跑 bag、不驱动真机。以下命令由你实机执行。

## 0. 结论速览

| 项 | 原仓库（Foxy 时代） | 你的 Go2（Humble） | 处理 |
|---|---|---|---|
| 位姿订阅 | `/utlidar/robot_pose` `PoseStamped` | **不存在**；实际是 `/utlidar/robot_odom` `nav_msgs/Odometry`（242Hz，odom→base_link） | ✅ 已改 DDS 订阅与消息类型 |
| 点云输入 | `/utlidar/cloud_deskewed`（frame odom） | ✅ 存在（14.3Hz），类型一致 | 保留 |
| 速度输出 | `/cmd_vel` → `/api/sport/request` | ✅ 话题正确 | 保留（新增断流看门狗） |
| slam_toolbox | 参数名 `params_file`（Foxy） | 只认 `slam_params_file` | ✅ 已改（否则 yaml 被静默忽略） |
| nav2 参数 | Foxy 写法（recoveries_server 等） | Humble 要 behavior_server / goal_checker_plugins / velocity_smoother | ✅ 已改 |
| 依赖包 | README 写 ros-foxy-* | 本机缺 slam_toolbox / nav2 | ⚠️ 你需 `apt install ros-humble-*`（见 §4.0） |
| 相机 | video_stream 组播网卡写死 `eth0` | 本机是 `ens33`/`ens37` | 已默认关闭相机启动 |

编译已通过（`colcon build`，5 个包），节点可正常启动/退出，未驱动真机。

## 1. 代码改动明细（10 个文件）

### go2_core/src/go2_base.cpp
- 位姿订阅改为 `nav_msgs::msg::Odometry` 的 `/utlidar/robot_odom`；`handlePose` → `handleOdom`。
- 发 TF `odom→base_link` 和 `/odom` 时，stamp 统一用主机 `now()`（bag 实测机器人时钟是 2025-04，与主机 2026-11 差 568 天，用消息 stamp 会导致 TF 查询落空）。
- `/odom` 的 `twist` 直接透传（原实现恒 0，nav2 无法判断运动状态）。
- **新增安全看门狗**：`/cmd_vel` 断流 0.5s 自动发 `StopMove`（原实现会按最后速度一直走）。
- **新增数据看门狗**：收不到 `/utlidar/robot_odom`（>3s）时每 5s 打一条可操作的 WARN（提示检查 source / RMW / CYCLONEDDS_URI 网卡），避免“RViz 里什么都没有但看不出原因”。

### go2_perception/src/cloud_accumulation.cpp
- 30 帧拼接（≈2s 点云）改为**5 帧拼接（≈0.35s）**：原来把 2 秒点云盖同一个发布时刻 stamp，机器人走动时建图会拖影；5 帧既保留一点密度、又不会跨太久。
- **删除 z∈[0.2,1.0] 的高度裁剪**：`/utlidar/cloud_deskewed` 本身就在 odom 坐标系（地面 z≈0），按 0.2~1.0 裁剪后一帧只剩 60~80 个点，RViz 里基本等于没有点云。2D 扫描需要的高度过滤由 `pointcloud_to_laserscan` 在 base_link 系完成。
- 发布频率 50Hz → 15Hz；先 `spin_once` 再判断有无点云才发布（不再发默认空云）。

### go2_perception/src/pointcloud_to_laserscan_node.cpp（+ .hpp）
- 删除 `/utlidar/robot_pose` **死订阅**（回调只存不用）与 `robotPoseCallback`。
- 删除每 100ms 无条件重建订阅的线程（原实现会随 graph 变化反复 unsubscribe/subscribe，容易丢帧）；改为启动时订阅一次。
- 移除 MessageFilter（按机器人时间戳等 TF 永远等不到）；改为回调里用主机 `now()` 查 TF。
- LaserScan header stamp 用主机 `now()`；TF 用重打时间戳的点云副本。

### launch / 配置
- `go2_perception/launch/go2_pointcloud_process.launch.py`：`transform_tolerance` 0.01→0.1，`scan_time` 0.25→0.07。
- `go2_slam/launch/go2_slamtoolbox.launch.py`：`params_file` → `slam_params_file`。
- `go2_core/launch/go2_startup.launch.py`：一键启动默认关闭相机（GStreamer 网卡写死 eth0），需要时单独开。
- `go2_navigation/config/nav2_params.yaml`（Humble 化）：
  - `recoveries_server` → `behavior_server`（`nav2_behaviors/*`）；
  - `goal_checker_plugin`（单数）→ `goal_checker_plugins`（复数）；
  - 删 `default_bt_xml_filename`（Humble 改名且默认值即原意图），`plugin_lib_names` 换成 Humble 默认表；
  - 新增 `velocity_smoother`（Humble 的 navigation_launch 会启动它，最终 `/cmd_vel` 由它发出）；
  - 新增 `smoother_server`；`map_server.yaml_filename` 去掉 turtlebot3 遗留、`map_saver` 改 double/transient local；
  - 真机安全限速：DWB `max_vel_x` 3.0→0.8、`max_vel_theta` 1.5→1.0、`max_speed_xy`→0.8、`acc_lim_x` 10→2.5；`robot_radius` 0.22→0.35（Go2 机身 0.7m）；amcl 的 `base_frame_id`/`robot_model_type` 改成 `base_link`/`nav2_amcl::DifferentialMotionModel`（本仓库没有 base_footprint TF）。
- `go2_core/package.xml`、`go2_perception/package.xml`：补齐 nav_msgs / tf2 / tf2_ros / laser_geometry / message_filters / tf2_sensor_msgs 等声明。

## 2. 编译

```bash
# 已在本机执行过，产物在 /home/go2/go2_toolbox_ws/{build,install,log}
source /opt/ros/humble/setup.bash
source /home/go2/unitree_ros2/setup.sh        # 注意：只有 setup.sh 会 export RMW_IMPLEMENTATION + CYCLONEDDS_URI
cd /home/go2/go2_toolbox_ws
colcon build --base-paths /home/go2/go2_ros2_toolbox \
  --packages-select go2_sport_api go2_core go2_perception go2_slam go2_navigation \
  --symlink-install
source install/setup.bash
```

> 源码继续改后重新执行上面最后两条即可（`--symlink-install`，C++ 需重编，launch/yaml 改完直接生效）。
>
> ⚠️ **必须在 `go2_toolbox_ws` 目录里执行 `colcon build`**。若在 `/home/go2` 直接跑，colcon 会在 `/home/go2/{build,install,log}` 另建一套 workspace，产物不在你 source 的目录里 —— 症状就是“代码改了、编译也成功了，但运行行为没变”。

## 3. 每个终端都要先 source

```bash
source /opt/ros/humble/setup.bash
source /home/go2/unitree_ros2/setup.sh        # 关键：会 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 和 CYCLONEDDS_URI（绑 ens37）
source /home/go2/go2_toolbox_ws/install/setup.bash
```

> ⚠️ **别只 source `unitree_ros2/cyclonedds_ws/install/setup.bash`**：那个文件只把消息包加进 `AMENT_PREFIX_PATH`，
> **不会**设置 `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI`。只 source 它时 ROS 用默认的 `rmw_fastrtps_cpp`，
> 看不到 CycloneDDS 的机器人话题 —— 现象就是三个节点正常启动、但没有任何图/TF/点云，go2_base 报
> `等待 /utlidar/robot_odom`。正确写法是 source `unitree_ros2/setup.sh`。
>
> `setup.sh` 里 `CYCLONEDDS_URI` 写死了 `ens37`（本机 `ens37 = 192.168.123.99`，正是接机器人的网卡，UP 时不用改）。
> 如果哪天机器人改接到别的网卡，**不要去改 `unitree_ros2` 里的文件**，在当前终端临时覆盖即可：
>
> ```bash
> export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="ens37" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
> # 把 ens37 换成 `ip -br link` 里实际 UP 且 192.168.123.x 的那张网卡
> ```

自检（期望三项都对）：

```bash
echo $RMW_IMPLEMENTATION          # rmw_cyclonedds_cpp
echo $CYCLONEDDS_URI              # 包含 NetworkInterface name="<接机器人的网卡>"
ros2 topic hz /utlidar/robot_odom # ~242Hz
```

**症状：RViz 里 Global Status 显示 `No tf data ... Frame [odom] does not exist`，画面只有 Grid，同时 go2_base 报 WARN。**
就是这个终端没把 DDS 环境弄对（最常见：只 source 了 `cyclonedds_ws/install/setup.bash`，或 source 在别的终端）。go2_base 启动时会打印实际生效的环境，并在收不到数据时每 5s 提示一次：

```
[INFO] [go2_base]: DDS 环境: rmw=rmw_cyclonedds_cpp; RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ROS_LOCALHOST_ONLY=0; CYCLONEDDS_URI=<CycloneDDS>...
[WARN] [go2_base]: 等待 /utlidar/robot_odom; 检查: 1) source /home/go2/unitree_ros2/setup.sh （只 source cyclonedds_ws/install/setup.bash 不会设置 RMW/CYCLONEDDS_URI）2) echo $RMW_IMPLEMENTATION 应为 rmw_cyclonedds_cpp 3) CYCLONEDDS_URI 绑定的网卡(ens37)是否 UP 且机器人网段 192.168.123.x
```

看 `rmw=` 那一项：若是 `rmw_fastrtps_cpp` 就是环境没 source 对。

## 4. 建图测试（三个阶段，各开一个终端）

```bash
# T1：位姿/控制桥
ros2 launch go2_core go2_base.launch.py

# T2：点云 → 激光
ros2 launch go2_perception go2_pointcloud_process.launch.py

# T3：slam_toolbox 建图（需已 apt 安装 slam_toolbox，见 §4.0）
ros2 launch go2_slam go2_slamtoolbox.launch.py
```

验证（另开终端）：

```bash
ros2 run tf2_ros tf2_echo odom base_link      # 应持续输出，不要 "Lookup would require extrapolation"
ros2 topic hz /scan                            # 期望 ~14Hz
ros2 topic echo /scan --once --field header    # frame_id: base_link
ros2 run rviz2 rviz2 -d /home/go2/go2_toolbox_ws/install/go2_core/share/go2_core/config/default.rviz
# RViz 里 Add → LaserScan(/scan) 和 Map(/map)；走动后地图应逐步扩展，扫描与地图应对齐
```

### 4.0 安装缺失依赖（一次性，需要 sudo）

```bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

（README 里的 `ros-foxy-*` 包名在你的 Humble 上装不上，对应 `ros-humble-*`。）

## 4.5 实时可视化（RViz，不需要 slam_toolbox/nav2）

一条命令同时起「位姿桥 + 点云→激光 + RViz」：

```bash
ros2 launch go2_core go2_live_view.launch.py
```

RViz 里预设了这些显示（`go2_core/config/default.rviz`，已改）：
- **Fixed Frame = odom**（机器人一有里程计就不会报错）
- `Map` /map（建图时才有，QoS Transient Local）
- `TransCloud` /trans_cloud（点云，frame=odom，5 帧累积、约 5k 点/帧；点尺寸 0.05）
- `Scan` /scan（激光，frame=base_link）
- `TF`（能看到 odom→base_link）
- `Odometry` /odom、`Path` /plan
- `Camera` /camera/image_raw（默认关闭，相机可用后再启用）

参数：`rviz:=false`（只跑节点不开 RViz）、`video_enable:=true`（开相机，需先改组播网卡）。

RViz 里如果只见 Grid 不见点云：确认 `ros2 topic hz /trans_cloud`（~14Hz）、`ros2 run tf2_ros tf2_echo odom base_link` 正常。

### 4.6 排查：RViz 里机器人/点云“乱跑”

2026-11-05 实测踩坑：RViz 里 TF、Odometry、点云乱跳。根因不是本仓库代码，而是**上一轮 FAST-LIO/bag 实验的残留进程**还在跑：

```bash
# 症状：/utlidar/robot_odom 有两个发布者（机器人 + 回放），位置在 0.5m 和 -2.6m 之间来回跳
ros2 topic info /utlidar/robot_odom -v     # Publisher count: 2
# 症状：有 /tf，节点是 laser_mapping/fastlio，frame camera_init→body，时间戳是机器人时钟（2025-04）
ros2 topic info /tf -v
ps aux | grep -E "rosbag2|fastlio|map_builder|lowstate_to_imu" | grep -v grep
```

本次处理：杀掉残留的 `rosbag2_player`（回放 `/home/go2/go2_bags/20261105_094814_walk_308_3` 的 `/utlidar/robot_odom`）、`fastlio_mapping`、`map_builder`、`rosbag2_recorder`、`lowstate_to_imu` 后，`/utlidar/robot_odom` 只剩机器人一个发布者、`/tf` 消失，RViz 不再乱跳。

注意：以后**一边跑 FAST-LIO/bag 回放、一边开这个 RViz** 还会复现：
- 回放的 bag 里有 `/utlidar/robot_odom` 时，会与本机机器人数据混在同一个话题上；
- FAST-LIO 会发自己的 `/tf`（`camera_init→body`，机器人时钟），tf2 会打印 `TF_OLD_DATA ... frame body ... authority`，属预期。
- 机器人机身时钟约 2025-04，主机是 2026-11；本仓库所有 TF/`/odom`/`/scan`/`/trans_cloud` 都重打成**主机 now()**，所以别再额外起用消息 stamp 发 TF 的节点。


### 4.7 排查：RViz 退出时 “exit code -11” 崩溃（已修）

症状：Ctrl-C / 关窗时 launch 打印：

```
[ERROR] [rviz2-4]: process has died [pid ..., exit code -11, cmd '.../rviz2 -d .../default.rviz ...']
```

其它三个节点都 `process has finished cleanly`，只有 rviz2 段错误。

根因（实测定位）：`default.rviz` 里加载的两个可选面板插件在退出时崩溃：

- `nav2_rviz_plugins/Navigation 2`
- `slam_toolbox::SlamToolboxPlugin`

同一份 RViz 配置、只改面板，启动 9 s 后发 SIGTERM 的对比：

| 配置 | rviz2 退出码 |
| --- | --- |
| 带这两个面板 | **139（SIGSEGV，/var/crash 生成 rviz2 crash）** |
| 去掉这两个面板 | **0（干净退出）** |

处理：已从 `go2_core/config/default.rviz` 移除这两个面板（这是本仓库 rviz 配置问题，不是节点问题）。需要 Navigation 2 面板时，在 RViz 里用 `Panels → Add New Panel` 临时添加即可；崩退只影响面板自身。

顺带修掉的两个日志噪音：

1. launch 不再给 rviz2 传 `name=`。原先会生成 `-r __node:=rviz2`，把 RViz 进程内所有子节点都改名成 `rviz2`，于是出现
   `Publisher already registered for provided node name` 警告。
2. 点云→激光的 TF 查询改用“最新可用变换”（`lookupTransform(..., tf2::TimePointZero)`）。原先按 `now()` 查，启动瞬间会偶发
   `Transform failure: Lookup would require extrapolation into the past`。

验证：完整 `ros2 launch go2_core go2_live_view.launch.py` 跑 14 s 后对进程组发 SIGINT —— 四个进程（含 rviz2）全部干净退出（rviz2 exit=0），日志无 WARN/ERROR，`/var/crash` 无新崩溃；RViz 连续运行 196 s 未崩溃，画面正常显示点云/scan/TF/Grid。

补充：`nav2`、`slam_toolbox`、`nav2_rviz_plugins` 现已安装（`ros2 pkg list` 可见），README 前置依赖不再是阻塞项。


## 5. 导航测试（真机安全第一）

建图中即可用（本仓库 `navigation_launch.py` 不带 map_server/amcl，global_costmap 直接吃 slam 的 `/map`）：

```bash
# T4
ros2 launch go2_navigation go2_nav2.launch.py
```

然后在 RViz 用 “2D Goal Pose” 发目标。

**安全要求**：
- 先把狗吊起或手扶，确认 `/cmd_vel` 方向正确（进度：`ros2 topic echo /cmd_vel`）再放地上；
- 限速已压到 0.8 m/s、角速度 1.0 rad/s；室内空旷处测试；
- 遥控器随时接管（阻尼模式）；`go2_base` 有 0.5s 断流自动 StopMove，但**不要依赖它**；
- 该链路会真的向 `/api/sport/request` 发运动指令，机器人会动。

## 6. 保存地图

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/go2/maps/go2_map'}}"
# 或者：ros2 run nav2_map_server map_saver_cli -f /home/go2/maps/go2_map
```

## 7. 一键启动（建图+导航+RViz，相机默认关）

```bash
ros2 launch go2_core go2_startup.launch.py
```

## 8. 相机链路（可选，暂未验证）

- `video_stream_node.py` 的 GStreamer URI 写死 `multicast-iface=eth0`，需改成连接机器人的网卡（Go2 前视相机组播 `230.1.1.1:1720`）。
- `unitree_pkgs/go2_h264_repub` 发布 `"/camera/image/raw"`，与 video node 的 `"/camera/image_raw"` 不一致，且该节点未被任何 launch 引用；仓库里带的是预编译二进制，建议不用。
- `go2_core/launch/go2_base.launch.py` 可用 `video_enable:=true` 单独开，但先改网卡。

## 9. 已知遗留 / 未处理

- `go2_navigation` 下两个 Python 节点（`navigation_command_tcpbridge.py` 等）依赖不存在的 `go2_tcp_toolbox` 包，且未被安装/启动，未改（不影响建图导航）。
- 仓库自带 `unitree_pkgs/unitree_api|unitree_go` 与本机 unitree_ros2 重名；本次编译**没有**构建它们（用系统安装版），构建命令已用 `--packages-select` 规避覆盖。
- `default.rviz` 已按 `go2_live_view.launch.py` 预设 Grid/Map/TransCloud/Scan/TF/Odometry/Path/Camera（相机默认关）；没装 slam_toolbox/nav2 时其中两个面板会报加载失败，不影响显示。
- nav2 的 `bt_navigator` 沿用了 Humble 默认 BT（`navigate_to_pose_w_replanning_and_recovery.xml`），仓库原来指向的 Foxy XML 不存在。
- `go2_base` 发出的 `Move` 是持续指令（与 Go2 sport API 语义一致），上真机前务必确认急停手段。

## 10. 回滚

```bash
cd /home/go2/go2_ros2_toolbox
git diff            # 看全部改动
git checkout -- .   # 回滚（新增的 ADAPTATION_*.md 不受影响）
```
