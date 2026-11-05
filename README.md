# Go2 ROS2 Toolbox（ROS2 Humble / CycloneDDS 适配版）

Unitree Go2（EDU）的 ROS2 工具箱：把 Go2 自带的激光雷达（L1/LiDAR）、里程计与 Navigation2 串成一条可用的 SLAM / 导航链路，包含点云累积、点云转激光扫描、slam_toolbox 建图、Nav2 导航、RViz 实时显示与 `/cmd_vel` → Unitree Sport API 的桥接。

本仓库基于 [andy-zhuo-02/go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox)（原版面向 Ubuntu 20.04 + ROS2 **Foxy**），已适配到：

| 项目 | 适配后 |
| --- | --- |
| 系统 | Ubuntu 22.04 |
| ROS2 | **Humble** |
| 中间件 | **CycloneDDS**（`rmw_cyclonedds_cpp`），经 unitree_ros2 连接机器人 |
| 机器人 | Go2 EDU（网口直连，机器人网段 `192.168.123.x`，本机网卡 `ens37`） |
| 已验证 | 编译、RViz 实时点云、slam_toolbox 建图（`/scan` 约 15 Hz）、Nav2 参数加载；实机动作为安全起见需用户自行上机验证 |

> 英文原版说明保留在上游仓库与 git 历史中；中文适配说明与改动细节见 [CHANGES_AND_TEST.md](./CHANGES_AND_TEST.md)、[ADAPTATION_REVIEW.md](./ADAPTATION_REVIEW.md)、[ADAPTATION_PLAN.md](./ADAPTATION_PLAN.md)。

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo" width="100%" style="background-color: #1a1a1a;">
</div>

## ✨ 功能

- **点云接入与累积**：订阅 `/utlidar/cloud_deskewed`（机器人直接在 `odom` 系发布去畸变点云），5 帧累积后发布 `/trans_cloud`。
- **点云 → 2D 激光扫描**：把 `/trans_cloud` 转成 `sensor_msgs/LaserScan`（`/scan`），供 slam_toolbox / Nav2 使用。
- **里程计与 TF**：订阅 `/utlidar/robot_odom`（`nav_msgs/Odometry`），发布 `/odom` 与 `odom → base_link` TF（统一使用主机时间戳）。
- **SLAM 建图**：slam_toolbox 在线异步建图，提供 `map → odom` TF 与 `/map`。
- **Nav2 导航**：完整的 Nav2（Humble 版参数）导航 + `velocity_smoother` 限速，最终输出 `/cmd_vel`。
- **运动桥接**：`/cmd_vel` 转换为 Unitree Sport API 请求（`/api/sport/request`），带断流保护。
- **RViz 实时显示**：一条 launch 即可看网格 / 点云 / 扫描 / TF / 里程计，不依赖 SLAM 与 Nav2。
- **排障友好**：启动即打印实际 DDS 环境；`/utlidar/robot_odom` 3 秒无数据、`/cmd_vel` 0.5 秒断流都会打印明确的警告/自动停车。

## 📁 目录结构

```text
go2_ros2_toolbox/
├── go2_core/                 # go2_base（位姿/TF/速度桥接）、video_stream_node.py（相机）
│   ├── config/default.rviz   # 适配后的 RViz 配置（odom 系、点云/扫描/TF/里程计）
│   └── launch/
│       ├── go2_base.launch.py       # 只起 go2_base（+可选相机）
│       ├── go2_live_view.launch.py  # 实时可视化：base + 点云处理 + RViz
│       └── go2_startup.launch.py    # 全链路：base + perception + slam + nav2 + rviz
├── go2_perception/           # cloud_accumulation、pointcloud_to_laserscan
├── go2_slam/                 # slam_toolbox 启动与建图参数
├── go2_navigation/           # Nav2 参数与启动
├── unitree_pkgs/             # unitree_api / unitree_go 消息 + go2_sport_api + go2_h264_repub
├── CHANGES_AND_TEST.md       # 逐文件改动、编译与上机测试命令、排障记录
├── ADAPTATION_REVIEW.md      # 原始仓库与本机环境的适配性检查报告
├── ADAPTATION_PLAN.md        # 适配方案（问题分层、补丁、验收标准）
└── README.md
```

## 🧰 环境要求

- Ubuntu 22.04 + ROS2 **Humble**（`/opt/ros/humble`）
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)（CycloneDDS 工作区，提供 `unitree_go / unitree_hg / unitree_api` 消息包）
- Go2 EDU 已开机并通过网线连接本机（本机网卡 `ens37`，与机器人 `192.168.123.x` 同网段）
- ROS2 依赖包：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
                 ros-humble-rviz2 ros-humble-cv-bridge ros-humble-laser-geometry \
                 ros-humble-message-filters ros-humble-rclcpp-components \
                 ros-humble-tf2-sensor-msgs ros-humble-rosbag2
```

> 只做“实时点云显示”时，`navigation2 / nav2-bringup / slam-toolbox` 不是必需的，其余仍需安装。

## 🛠️ 编译

本仓库不要求放在 `colcon` 工作区的 `src/` 下（`go2_core` 用相对路径引用 `unitree_pkgs/go2_sport_api/include`，只要仓库目录保持完整即可）。本机验证过的布局：

```bash
# 仓库：/home/go2/go2_ros2_toolbox
# 工作区：/home/go2/go2_toolbox_ws（build/install/log 都生成在这里）

source /opt/ros/humble/setup.bash
source /home/go2/unitree_ros2/setup.sh          # 见下方“环境”一节，必须先 source 它
cd /home/go2/go2_toolbox_ws
colcon build --base-paths /home/go2/go2_ros2_toolbox \
             --packages-select go2_sport_api go2_core go2_perception go2_slam go2_navigation \
             --symlink-install
source install/setup.bash
```

要点：

- **务必在 `go2_toolbox_ws` 里执行 `colcon build`**（而不是在 `/home/go2` 或仓库根目录），否则产物会写到别处的 `build/install`，出现“改了代码但不生效”。
- `unitree_api / unitree_go` 这两个消息包建议**不要**一起编译（本机已由 unitree_ros2 安装），避免 overlay 覆盖；命令用 `--packages-select` 只编译 5 个包即可。
- `--symlink-install` 下修改 launch / rviz / yaml 无需重编；修改 C++ 需要重新 `colcon build`。

## ⚠️ 环境（最常见的坑）

**只 source `unitree_ros2/cyclonedds_ws/install/setup.bash` 是不够的**——它只把消息包加到 `AMENT_PREFIX_PATH`，**不会**设置 `RMW_IMPLEMENTATION` 与 `CYCLONEDDS_URI`，此时 RMW 会退回 `rmw_fastrtps_cpp`，表现为：

- `go2_base` 一直打印 `等待 /utlidar/robot_odom / 3s 未收到新的 /utlidar/robot_odom`
- RViz 里 `No tf data`、没有点云

正确做法（每个新终端都执行）：

```bash
source /home/go2/unitree_ros2/setup.sh                      # 设 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 与 CYCLONEDDS_URI(ens37)
source /home/go2/go2_toolbox_ws/install/setup.bash
```

`unitree_ros2/setup.sh` 实际做的事：

```bash
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="ens37" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

如果本机网卡不是 `ens37`：**不要修改 unitree_ros2 里的文件**，在当前终端临时覆盖即可：

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

### 环境自检

```bash
echo $RMW_IMPLEMENTATION        # 应为 rmw_cyclonedds_cpp
echo $CYCLONEDDS_URI            # 应包含 <NetworkInterface name="ens37" ...>
ip -br addr show ens37          # 应为 UP，且是 192.168.123.x
ros2 topic hz /utlidar/robot_odom   # 应约 240 Hz
ros2 topic info -v /utlidar/robot_odom   # Publisher count 应为 1（见 FAQ：残留 rosbag 回放）
```

`go2_base` 启动时会打印一行 `DDS 环境: rmw=...; RMW_IMPLEMENTATION=...; ROS_LOCALHOST_ONLY=...; CYCLONEDDS_URI=...`，看到 `rmw=rmw_cyclonedds_cpp` 就说明环境正确。

## 🚀 运行

### 1. 实时可视化（不需要 SLAM / Nav2，最快自检）

```bash
ros2 launch go2_core go2_live_view.launch.py
```

会启动 `go2_base`、`cloud_accumulation`、`pointcloud_to_laserscan` 与 RViz（配置 `go2_core/config/default.rviz`）：

- `Fixed Frame = odom`，显示 Grid、`/trans_cloud`、`/scan`、TF、`/odom`；
- 机器人静止时：点云（约 5k 点/帧，15 Hz）、红色 2D 扫描线、`odom → base_link` TF、里程计都正常显示。

可选参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `rviz` | `true` | `rviz:=false` 只跑节点不起界面 |
| `video_enable` | `false` | 起相机节点（见“相机”一节） |
| `image_topic` | `/camera/image_raw` | 相机话题 |

```bash
# 无界面跑一遍链路（排查用）
ros2 launch go2_core go2_live_view.launch.py rviz:=false

# 单独看数据
ros2 topic hz /trans_cloud
ros2 topic hz /scan
ros2 tf2_echo odom base_link
```

### 2. 建图（SLAM）

```bash
ros2 launch go2_core go2_startup.launch.py
```

依次启动：`go2_base` → 点云处理 → slam_toolbox → Nav2 → RViz（相机默认关闭）。建图完成后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/go2_map
```

> 本仓库的 `default.rviz` 只保留 `rviz_common` 面板（slam/nav2 面板在退出时会触发 -11 崩溃），
> 因此没有 *Save Map* 按钮；保存地图请用上面的命令行，或用 slam_toolbox 服务（`/slam_toolbox/save_map`）。

也可以分步启动（便于单独排查）：

```bash
ros2 launch go2_core go2_base.launch.py
ros2 launch go2_perception go2_pointcloud_process.launch.py
ros2 launch go2_slam go2_slamtoolbox.launch.py
```

> slam_toolbox（Humble）的参数名是 `slam_params_file`，本仓库 launch 已改好；若沿用 Foxy 时代的 `params_file`，参数会被静默忽略。

### 3. 导航（Nav2）

```bash
ros2 launch go2_navigation go2_nav2.launch.py
```

- 参数文件：`go2_navigation/config/nav2_params.yaml`（已按 Humble 结构：`behavior_server`、`velocity_smoother`、Humble BT 插件列表、`robot_radius 0.35`）。
- 限速：`FollowPath` 最大速度 0.8 m/s、`velocity_smoother` 上限 `[0.8, 0, 1.0]`；真机首次测试建议手扶/悬空并随时用遥控器接管。
- 该 launch 只包含 `navigation_launch.py`（控制器/规划器/行为/BT），**不含** `map_server` 与 `amcl`；请用 slam_toolbox 建图（`/map` 由 slam 提供），或自行启动地图服务与定位。

### 4. 单独节点

```bash
ros2 run go2_core go2_base                    # 位姿/TF/速度桥接
ros2 run go2_perception cloud_accumulation    # /utlidar/cloud_deskewed -> /trans_cloud
ros2 run go2_perception pointcloud_to_laserscan_node   # /trans_cloud -> /scan（组件也可用）
```

### 5. 相机（可选，默认关闭）

```bash
ros2 launch go2_core go2_live_view.launch.py video_enable:=true
```

相机节点用 GStreamer 拉 `udp://230.1.1.1:1720` 组播 H.264，脚本里 `multicast-iface=eth0` 是写死的；本机没有 `eth0` 时需按实际网卡修改 `go2_core/go2_core/video_stream_node.py`。`unitree_pkgs/go2_h264_repub` 是上游的另一个相机节点，发布话题名为 `/camera/image/raw`，本仓库未使用。

## 📡 话题一览（Go2 实机实测）

### 机器人提供、本仓库使用

| 话题 | 类型 | 频率 | Frame | 说明 |
| --- | --- | --- | --- | --- |
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | ~240 Hz | `odom` → `base_link` | **本仓库唯一位姿源**（原版用的 `/utlidar/robot_pose` 在实机上不存在） |
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | ~14.3 Hz | `odom` | 已去畸变点云（部分录制包里没有该话题，但机器人运行时持续发布） |
| `/cmd_vel` | `geometry_msgs/Twist` | — | `base_link` | 速度指令（Nav2 输出） |

> 机器人自带的 `/utlidar/robot_pose`（PoseStamped）与 LIO-SAM 相关话题（`/tf`、`/Odometry`、`/cloud_registered` 等）本仓库**不使用**，也不要和本仓库的 `odom → base_link` TF 混用。

### 本仓库发布

| 话题 | 类型 | 频率 | Frame | 发布者 |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | ~240 Hz | `odom` → `base_link` | `go2_base`（位姿透传，twist 透传） |
| `/tf` | `tf2_msgs/TFMessage` | ~240 Hz | `odom → base_link` | `go2_base`（时间戳统一为主机 now()） |
| `/trans_cloud` | `sensor_msgs/PointCloud2` | ~15 Hz | `odom` | `cloud_accumulation`（5 帧累积） |
| `/scan` | `sensor_msgs/LaserScan` | ~15 Hz | `base_link` | `pointcloud_to_laserscan` |
| `/api/sport/request` | `unitree_api/msg/Request` | 按需 | — | `go2_base`（Move / StopMove） |

### 数据流

```text
Go2 内置 L1 激光雷达
   │  /utlidar/cloud_deskewed (PointCloud2, frame=odom, ~14Hz)
   ▼
cloud_accumulation ──/trans_cloud (PointCloud2, frame=odom, ~15Hz)──► RViz
   ▼
pointcloud_to_laserscan ──/scan (LaserScan, frame=base_link, ~15Hz)──► slam_toolbox ──/map, map→odom──► Nav2
                                                                           ▲
Go2 里程计 /utlidar/robot_odom (Odometry, odom→base_link, ~240Hz)          │
   ▼                                                                       │
go2_base ──/odom + TF(odom→base_link)──────────────────────────────────────┘
   ▲
   └── /cmd_vel (Twist) ◄── Nav2 (velocity_smoother)
go2_base ──/api/sport/request (unitree_api/Request)──► Go2 Sport API（真机运动）
```

## ⚙️ 关键参数

| 位置 | 参数 | 值 | 说明 |
| --- | --- | --- | --- |
| `go2_perception/launch/go2_pointcloud_process.launch.py` | `target_frame` | `base_link` | 扫描所在坐标系 |
| 同上 | `min_height` / `max_height` | `-0.1` / `1.0` | 在 `base_link` 系按高度截取（过滤地面与头顶） |
| 同上 | `angle_increment` / `scan_time` | `0.0087` / `0.07` | 360° 约 722 束 |
| 同上 | `range_min` / `range_max` | `0.10` / `20.0` | 与雷达量程/地图参数一致 |
| `cloud_accumulation`（源码） | 累积帧数 | 5 | 约 0.35 s；`/trans_cloud` 发布 15 Hz |
| `go2_base`（源码） | `/cmd_vel` 看门狗 | 0.5 s | 断流自动 StopMove |
| `go2_base`（源码） | `/utlidar/robot_odom` 看门狗 | 3 s | 无数据时打印排障警告 |
| `go2_navigation/config/nav2_params.yaml` | `max_vel_x` / `velocity_smoother` | `0.8` / `[0.8, 0, 1.0]` | 安全限速（原版 3.0，太大） |
| 同上 | `robot_radius` | `0.35` | Go2 机身半径（原版 0.22 是 turtlebot 的） |
| `go2_slam/config/mapper_params_online_async.yaml` | `minimum_time_interval` | `0.5` | 建图 2 Hz；点云稀疏时可调到 0.2 |

## 🧭 排障（FAQ）

**1. RViz 里没有点云 / `No tf data` / `Frame [odom] does not exist`**

九成是 DDS 环境没弄对（只 `source cyclonedds_ws/install/setup.bash`）。按“环境”一节 source `unitree_ros2/setup.sh`，并确认 `go2_base` 启动日志是 `rmw=rmw_cyclonedds_cpp`；再确认 `ros2 topic info -v /utlidar/robot_odom` 的 Publisher count = 1。

**2. `rviz2` 退出时 `exit code -11`（段错误）**

上游 RViz 配置里的 `nav2_rviz_plugins/Navigation 2` 与 `slam_toolbox::SlamToolboxPlugin` 面板在退出时会崩（不影响显示，但会留下 -11 和 core）。本仓库的 `go2_core/config/default.rviz` 已去掉这两个面板；请使用本仓库的配置，不要用上游旧配置。

**3. 点云很稀疏 / 几乎看不见**

`cloud_accumulation` 最初还做了 `z ∈ [0.2, 1.0]` 截断，在 `odom` 系里把地面和低处几乎全滤掉（每帧只剩几十点）。现已改为 5 帧累积 + 不在 3D 阶段截高度（高度过滤交给 `pointcloud_to_laserscan` 在 `base_link` 系做），RViz 里点尺寸 0.05。

**4. RViz 里机器人/点云“乱跳/乱跑”**

通常是图里还有别的进程在抢发同一个话题：最常见是残留的 `rosbag2_player`（回放旧 bag 里的 `/utlidar/robot_odom`）或上位机自己的 FAST-LIO 在发 `/tf`。检查：

```bash
ros2 topic info -v /utlidar/robot_odom    # Publisher count 必须是 1
ros2 topic list | grep -x /tf             # 不应有机器人 LIO 的 TF 抢帧
```

**5. `/cmd_vel` 发了机器人不动**

- 确认 `go2_base` 在运行（它才是 `/cmd_vel` → `/api/sport/request` 的桥）；
- 确认 DDS 环境正确、`ros2 topic info -v /api/sport/request` 能看到发布者；
- 查看 `go2_base` 日志是否有 `No /cmd_vel for 0.5s, sending StopMove`（0.5 s 没指令会自动停车，属于安全保护）。

**6. 时间戳对不上 / `TF_OLD_DATA` / `extrapolation` 警告**

机器人内部时钟与主机可能相差数月（实测 2025-04 vs 2026-11）。本仓库统一在 `go2_base` 与 `cloud_accumulation` / `pointcloud_to_laserscan` 中用主机 `now()` 重新打时间戳，并用“最新可用变换”查 TF；正常情况下不应再报 TF 相关错误。

**7. `slam_toolbox` / `nav2` 启动报 `PackageNotFoundError`**

Humble 下需要安装：`sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup`。

## ⚠️ 安全

本仓库的 `/cmd_vel` 会**真实驱动机器人**（经 `/api/sport/request`）。第一次上机时：

1. 手扶机器人或让它悬空/趴下，先只跑 `go2_live_view.launch.py` 验证数据链路；
2. 跑 Nav2 前把机器人放在空旷处，限速已降到 0.8 m/s；
3. 遥控器保持可接管（进入遥控模式会覆盖 sport API 指令）；
4. 看到异常立即 Ctrl-C：`go2_base` 会在 0.5 s 无指令后自动 StopMove，退出时也会发停。

## 📄 文档索引

| 文档 | 内容 |
| --- | --- |
| [CHANGES_AND_TEST.md](./CHANGES_AND_TEST.md) | 逐文件改动、编译命令、各场景运行与验证命令、踩坑记录（推荐先看这个） |
| [ADAPTATION_REVIEW.md](./ADAPTATION_REVIEW.md) | 原始仓库与本机 Go2/Humble 环境的适配性检查：话题/消息类型/参数逐项核对 |
| [ADAPTATION_PLAN.md](./ADAPTATION_PLAN.md) | 适配方案：问题分层（P0/P1/P2）、补丁清单、验收标准 |

## 📄 License

MIT，见 [LICENSE](./LICENSE)。本项目为个人适配版本，与 Unitree Robotics 无隶属关系；原始项目版权归 [andy-zhuo-02/go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox) 作者所有。

## 🙏 致谢

- [andy-zhuo-02/go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox)：原始工具箱与 RViz/launch 结构
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)：Unitree 消息与 CycloneDDS 接入
- ROS2、slam_toolbox、Navigation2 社区
