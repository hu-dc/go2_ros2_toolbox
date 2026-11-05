# go2_ros2_toolbox 适配方案（Humble + Go2 实机）

> 环境：Ubuntu 22.04 / ROS 2 Humble + `/home/go2/unitree_ros2`（cyclonedds，setup.sh 绑 ens37）
> 仓库：`andy-zhuo-02/go2_ros2_toolbox` @ `cd11d74`（README 自述 Ubuntu 20.04 + Foxy，Go2 EDU 扩展坞，固件 1.1.7）
> 证据来源：22 个 `/home/go2/go2_bags/*` 的 metadata + db3 原始 CDR 解码；`/home/go2/unitree_ros2` 源码；Humble 的 nav2/slam_toolbox 上游源码
> 本文只出方案，未动仓库代码。

---

## 0. 结论

1. **不适配的根因只有一条：仓库订阅 `/utlidar/robot_pose`（PoseStamped），本机机器人不存在该话题。**
   机器人真实位姿话题是 `/utlidar/robot_odom`，类型 `nav_msgs/msg/Odometry`（242 Hz，frame `odom` → `base_link`，pose 和 twist 都带值）。
   仓库的整条 SLAM 链路（TF `odom→base_link`、`/odom`、`/scan`、slam_toolbox、nav2）全部依赖这一条。
2. `unitree_ros2` 里**没有** `/utlidar/robot_pose`、也没有任何 odom→TF 节点，更没有 `nav_msgs/Odometry` 的定义（`nav_msgs` 是 ROS 2 标准包，本机已装）。它是机器人固件经 DDS 直接发布的 lidar 话题，`unitree_ros2` 只提供 3 个消息包 + 示例。所以**不需要新增/自定义消息类型，订阅时用 `nav_msgs::msg::Odometry` 即可**。
3. 除话题外，还有 3 类次生问题：缺 `slam_toolbox` / `nav2` 包（本机 `/opt/ros/humble` 里都没有）、Humble 下 `slam_toolbox` 的参数名变了（`params_file` → `slam_params_file`）、以及仓库里有几处 Foxy 遗留/死代码会影响出图质量。
4. **建议方案 A（改仓库 3 个源文件 go2_base / cloud_accumulation / pointcloud_to_laserscan + 3 处 launch 参数 + 装 3 个包）**，约 1~2 小时改动 + 编译验证；方案 B（零侵入 bridge 节点）作为备选见 §9。

---

## 1. unitree_ros2 核实（你要求我自己查的部分）

检查对象：`/home/go2/unitree_ros2`，HEAD `668d1ec`（v0.3.0），含 `cyclonedds_ws/`、`example/`、`docs/`（仅图片）、`setup*.sh`、README。

### 1.1 unitree_ros2 提供了什么

- 消息包：`unitree_go`、`unitree_hg`、`unitree_api`（Go2 用 `unitree_go` + `unitree_api`）。
- 示例：`read_low_state.cpp`、`read_motion_state.cpp`（订阅 `lf/sportmodestate`）、`record_bag.cpp`、`go2/*`（sport client / state client / stand example）等。
- **没有** odom→TF 发布节点，**没有** 订阅 `/utlidar/robot_odom` 的示例。全仓库只有 `read_motion_state.cpp:36-37` 注释提到 “Robot position (Odometry frame)”。
- README 里 lidar 部分只写了 `/utlidar/cloud`（frame `utlidar_lidar`），**没有** `robot_pose` / `robot_odom` 的文档。
- 消息定义里与位姿相关的是 `SportModeState`（`position[3]` / `velocity[3]` / `yaw_speed` / `imu_state`，20 Hz，`TimeSpec` 不是 ROS header）；`LowState.imu_state`（机身 IMU，无 `header.stamp`）。

### 1.2 本机机器人话题真值（22 个 bag 的 metadata + CDR 解码）

| 话题 | 类型 | 频率 | frame / 备注 |
|---|---|---|---|
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 242 Hz | `odom` → `base_link`，pose.position≈(4.38,-3.86,0.32)，四元数≈单位，twist 有值 |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 14.3 Hz | frame `odom`，x/y/z/intensity，point_step=16（去畸变后的云） |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 14.7 Hz | frame `utlidar_lidar`，6 个 field，point_step=32 |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 249.6 Hz | `utlidar_imu`（L1 内置 IMU） |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | 20 Hz | 备用位姿源（无四元数，只有 yaw 速度） |
| `/lowstate`、`/lf/lowstate` | `unitree_go/msg/LowState` | 500 / 20 Hz | 机身 IMU 在 `imu_state`，无 header.stamp |

**22 个 bag 中 `/utlidar/robot_pose` 出现 0 次**；`/tf`、`/odom`、`/cmd_vel`、`/camera/*` 也都没有（这些都是仓库自己节点发布的）。

> 说明：bag 里的机器人时间戳是机身时钟（≈2025-04-15），比主机（2026-11-05）早 568 天 —— 这就是为什么要做 §4.4 的时间戳处理，否则 TF 查询必然失败。

---

## 2. 仓库话题逐项核对

| 节点 | 仓库使用的话题/类型 | 适配 | 说明 |
|---|---|---|---|
| `go2_base` | 订阅 `/utlidar/robot_pose` (PoseStamped) | ❌ | 应为 `/utlidar/robot_odom` (Odometry)，**必须改代码**（消息类型不同，remap 无效） |
| `go2_base` | 订阅 `/cmd_vel` (Twist) | ✅ | Humble nav2 最终也发 `/cmd_vel`（velocity_smoother） |
| `go2_base` | 发布 `/odom` (Odometry) | ✅ | nav2 的 `odom_topic: /odom` 对得上 |
| `go2_base` | 发布 `/api/sport/request` (unitree_api/Request) | ✅ | 与 unitree_ros2 的 SportClient 一致（真机会动，见安全说明） |
| `cloud_accumulation` | 订阅 `/utlidar/cloud_deskewed` | ✅ | 机器人确实发布，frame 恰为 `odom` |
| `pointcloud_to_laserscan` | 订阅 `/trans_cloud` + `/utlidar/robot_pose` | ✅ / ❌ | `/trans_cloud` 由 accumulation 发；`robot_pose` 订阅是死代码（回调只存不用） |
| `pointcloud_to_laserscan` | 发布 `/scan` | ✅ | 需 TF `odom→base_link`，这是被 §4.1 打断的那一环 |
| `slam_toolbox` | 订阅 `/scan` | ✅ | 参数名需改（§4.5） |
| `nav2` | 订阅 `/scan`、`/odom`、`/cmd_vel` | ✅ | 参数有 Foxy 遗留（§5.2） |
| `video_stream_node` | 发布 `/camera/image_raw` | ⚠️ | 无消费者；GStreamer 组播接口写死 `eth0`，本机网卡是 ens33/ens37 |
| `go2_h264_repub` | 发布 `/camera/image/raw` | ⚠️ | 拼写与 `/camera/image_raw` 不一致、iface 参数默认 `etho0`、无 launch 引用、仓库里还带预编译二进制 → 建议不用 |
| `navigation_command_tcpbridge.py` | 依赖包 `go2_tcp_toolbox` | ❌ | 仓库里没有这个包，且 `go2_navigation/CMakeLists.txt` 不安装 python → 必崩，建议删 |

---

## 3. 方案分层

| 层 | 内容 | 目的 |
|---|---|---|
| **P0 必改（链路通）** | 4 处代码/launch + 装 3 个 apt 包 | 让 `/odom`、TF、`/scan`、`/map`、nav2 全部跑起来 |
| P1 建议（质量/安全） | 感知节点清理、nav2 参数 Humble 化、启动顺序、速度限制 | 避免 2 s 陈旧点云、避免 DWB 3 m/s 直接下发真机 |
| P2 清理（可选） | package.xml 依赖、README 话题表、死文件、vendored 消息包 | 可维护性 |

---

## 4. P0 具体改动

### 4.0 装包（apt 已确认可装：navigation2 1.1.20 / slam-toolbox 2.6.10）

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

### 4.1 `go2_core/src/go2_base.cpp`：位姿订阅换成 `/utlidar/robot_odom`

改 4 处：

```cpp
// :31-35  订阅
odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/utlidar/robot_odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { handleOdom(msg); });

// :53-81  handlePose → handleOdom（TF 与 /odom 都用主机时钟，规避机身时钟差 568 天）
void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();          // 关键：主机时钟
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation      = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header   = t.header;
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose     = msg->pose;         // 整块透传（含协方差）
    odom_msg.twist    = msg->twist;        // 原实现 twist 恒 0，这里直接给真值
    odom_pub_->publish(odom_msg);
}

// :110 成员类型
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
```

同时 `:2` 的 `pose_stamped.hpp` 可删（`nav_msgs/msg/odometry.hpp` 在 `:7` 已经 include）。

### 4.2 `go2_perception/src/cloud_accumulation.cpp`：30 帧拼接 → 单帧透传

当前 `:18-21` 保留最近 30 帧再拼（14.3 Hz × 2 s ≈ 2 s 的点云），但时间戳在 `:87-88` 统一改成发布时刻 → 2 秒前的几何配当前 TF，建图会“拖影”。

```cpp
// :20   30 → 1（只做“重打时间戳 + 统一 frame_id=odom”的转发）
if (clouds.size() > 1) { clouds.erase(clouds.begin()); }
```

```cpp
// :88 附近：50 Hz 重复发布同一帧没有意义，改成 15 Hz；且首帧前不要发空云
rclcpp::Rate rate(15.0);
while (rclcpp::ok()) {
  if (!clouds.empty()) {                     // 加空数据守卫
    accumulated_cloud.header.stamp = node->get_clock()->now();
    pub->publish(accumulated_cloud);
  }
  executor.spin_some();
  rate.sleep();
}
```

改完后 `/trans_cloud` = 去畸变后的单帧云，frame `odom`，时间戳 = 主机 now（这一点很重要：`pointcloud_to_laserscan` 的 TF 查询和 `/scan.header.stamp` 都随它走）。

### 4.3 `go2_perception/src/pointcloud_to_laserscan_node.cpp`：删死订阅 + 去掉对 TF 滤波器的硬依赖

- 删除 `:85-88` 的 `/utlidar/robot_pose` 订阅，以及 `.hpp` 里 `robot_pose_sub_` / `pose_stamped` 成员；`src:153-157` 的 `robotPoseCallback` 一并删除。
- 构造函数里的 `MessageFilter`（`:92-106`）改为直接订阅；回调里显式用主机 now() 打时间戳再做 TF 查询。这样即使以后直接喂机器人的原始云（机器人时钟）也能工作：

```cpp
// 构造函数（target_frame 非空时）：保留 tf2_ Buffer/Listener，但不再用 MessageFilter
sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, std::placeholders::_1));

// cloudCallback 开头：
rclcpp::Time stamp = this->now();
auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
scan_msg->header.stamp    = stamp;
scan_msg->header.frame_id = target_frame_.empty() ? cloud_msg->header.frame_id : target_frame_;

// TF 部分（transform 用主机时刻，而不是机器人 stamp）：
if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
  try {
    sensor_msgs::msg::PointCloud2 in = *cloud_msg;   // 拷贝后重打时间戳
    in.header.stamp = stamp;
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    tf2_->transform(in, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
    cloud_msg = cloud;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
    return;
  }
}
```

- 另外把 `subscriptionListenerThreadLoop`（`:118-151`）删掉或恢复“有订阅者才订阅”的守卫：现在它每 100 ms 无条件 `sub_.subscribe(...)` 重建订阅（原代码的 guard 被注释掉了），容易丢消息。

### 4.4 `go2_perception/launch/go2_pointcloud_process.launch.py`

```python
'transform_tolerance': 0.1,   # 原 0.01：主机/TF 流 242Hz，稍微放宽，避免偶发 Transform failure
'scan_time': 0.07,            # 原 0.25：与实际 14~15Hz 云对齐（LaserScan 的 scan_time 字段）
```

其余参数（`target_frame=base_link`、`min_height=-0.1`、`max_height=1.0`、`angle ±3.14`、`range 0.10~20.0`）与 Go2 数据相容，不用改。
`base_link` 离地约 0.32 m（robot_odom z=0.32），`-0.1 ~ 1.0` 的高度带正好覆盖地面障碍。

### 4.5 `go2_slam/launch/go2_slamtoolbox.launch.py`：参数名

Humble 的 `slam_toolbox/launch/online_async_launch.py` 只声明 `use_sim_time` 和 `slam_params_file`；仓库传的是 `params_file` → **会被静默忽略**（退化成 slam_toolbox 默认 yaml，能用但不是你的参数）。

```diff
-            'params_file': slam_toolbox_config,
+            'slam_params_file': slam_toolbox_config,
```

### 4.6 `go2_core/launch/go2_startup.launch.py`：先关摄像头

`:29` 的 `image_topic=/camera/image_raw` 没有消费者；`video_enable` 默认 true 会启 `video_stream_node`，其 GStreamer 固定 `multicast-iface=eth0`，本机没有 eth0（是 ens33/ens37），大概率报错刷屏。先改成 `video_enable false`，等相机链路单独验证时再开。

`go2_startup.launch.py` 还建议拆成两个入口（P1）：`startup_slam`（base + perception + slam）与 `startup_nav`（base + nav2），建图和导航分开验证。

---

## 5. P1（建议同批或紧随）

1. **nav2 参数 Humble 化**（`go2_navigation/config/nav2_params.yaml`）：
   - 新增 `velocity_smoother` 段（默认 `max_velocity [0.50, 0, 2.5]` 会把你 DWB 的 3.0 m/s 砍到 0.5；显式写 `[0.5, 0.0, 1.5]` 之类更清楚）。
   - `recoveries_server` → Humble 的 `behavior_server`（插件名 `nav2_behaviors/Spin|BackUp|Wait`，`recovery_plugins`→`behavior_plugins`，`back_up`→`backup`）。不改的话整段被忽略、用默认行为，也能跑。
   - `default_bt_xml_filename`（`:59`）在 Humble 已改名 `default_nav_to_pose_bt_xml`（或干脆删掉用默认）；`plugin_lib_names` 是 Foxy 列表，**建议整段删掉**，用 Humble 默认完整列表，避免以后升级踩缺失插件。
   - `map_server`（`:238`）`yaml_filename: turtlebot3_world.yaml` 是 turtlebot3 遗留，仓库也不带地图文件；`navigation_launch.py` 不启动 map_server，先删/留空。
   - `amcl` 段同样不会启动（建图模式用 slam_toolbox 出 `/map`），可删。
   - 安全值：`max_vel_x 3.0` / `max_speed_xy 3.0` 对建图+避障太快，建议 0.6~0.8；`robot_radius 0.22` 对 Go2（机身约 0.7×0.31 m）偏小，建议 0.35 或换 `footprint`。
2. **rviz 配置**：`go2_core/config/default.rviz` 只有 `/map /odom /plan /initialpose`，加 `/scan`、`/trans_cloud`、TF。
3. **启动验证顺序**：先 `go2_base` → `/odom`+TF，再 perception → `/scan`，再 slam → `/map`，最后 nav2。
4. **slam 参数**：`minimum_time_interval: 0.5`（2 Hz 建图）+ `minimum_travel_distance: 0.5` 对慢速建图偏保守，可先保持，出图慢再调。
5. **`go2_core/package.xml`** 补 `nav_msgs`/`tf2_ros`/`tf2` 依赖声明（CMake 里已有，rosdep 会漏）。

## 6. P2（可选清理）

- 删 `go2_navigation` 里的 `navigation_command_tcpbridge.py`（依赖不存在的 `go2_tcp_toolbox`）和未安装的 python 配置；`tcp_client_standalone.py` 只是示例。
- `go2_h264_repub` 的预编译二进制不建议进仓库；源码的 `/camera/image/raw` 与 `etho0` 都要修（但目前无消费者）。
- README 话题表（`README.md:113-114` / `README_zh.md:120-121`）把机器人话题列在 “Publishers”，且写了不存在的 `/utlidar/robot_pose`，建议更正为机器人侧订阅源 + `robot_odom`。
- `unitree_pkgs/unitree_api`、`unitree_pkgs/unitree_go` 与本机已装的 `unitree_ros2` 重名（msg 定义一致，仅 `Go2FrontVideoData.msg` 差行尾空格）；同 overlay 编译会覆盖安装版，不致命。想干净就删这两个副本、只保留 `go2_sport_api`（依赖已装版本）。

---

## 7. 构建 / 运行 / 验证

### 7.1 目录与编译

仓库不能直接 `colcon build` 后从仓库目录跑吗？可以，但推荐用 `--base-paths` 把 build/install 放在仓库外：

```bash
cd /home/go2
source /home/go2/unitree_ros2/setup.sh          # 注意：CYCLONEDDS_URI 现在绑 ens37（当前 DOWN）
colcon build --base-paths go2_ros2_toolbox --symlink-install
source install/setup.bash
```

说明：`go2_core/CMakeLists.txt` 用 `include_directories(../unitree_pkgs/go2_sport_api/include)`，相对 `go2_core` 源目录解析，所以仓库根保持原样即可；`unitree_pkgs` 下 4 个包都会被 colcon 发现并按依赖顺序编译。

### 7.2 上机前

1. 网线接 Go2（当前 ens37 `NO-CARRIER`）。若机器人网口对应的是别的网卡名，改 `unitree_ros2/setup.sh` 里的 `CYCLONEDDS_URI` 网卡。
2. `ros2 topic list | grep utlidar` 应看到 `robot_odom`、`cloud_deskewed` 等；
   `ros2 topic info /utlidar/robot_odom -v` 应显示 `nav_msgs/msg/Odometry`。
   （如果这台机器的固件/配置真的还发 `/utlidar/robot_pose`，那说明话题表要再确认，方案 A 继续用 robot_odom 也不影响。）

### 7.3 分步验证

```bash
# 1) 位姿 + TF
ros2 run go2_core go2_base
ros2 topic hz /odom                     # ≈242Hz
ros2 topic echo /odom --once            # pose/twist 有值
ros2 run tf2_ros tf2_echo odom base_link

# 2) 点云转 scan
ros2 launch go2_perception go2_pointcloud_process.launch.py
ros2 topic hz /scan                     # ≈15Hz
ros2 topic echo /scan --field ranges | head   # 有有限距离

# 3) 建图
ros2 launch go2_slam go2_slamtoolbox.launch.py
ros2 topic hz /map
rviz2 -d src/go2_ros2_toolbox/go2_core/config/default.rviz

# 4) 导航（先别让狗动：不开 go2_base 的 /cmd_vel 转发或把狗吊起）
ros2 launch go2_navigation go2_nav2.launch.py
ros2 lifecycle get /controller_server
```

### 7.4 安全

`go2_base` 收到 `/cmd_vel` 会立即通过 `/api/sport/request` 让真狗运动；nav2 一启动就会发 cmd_vel。
**先只跑 1~3 步（base/perception/slam）确认建图，再单独开 nav2；首次开 nav2 时机器狗吊起或空旷低速，`max_vel_x` 先给 0.3。**

---

## 8. 验收标准

- [ ] `/odom` ≈242 Hz，pose + twist 有效；TF `odom→base_link` 可查（`tf2_echo` 稳定不断）
- [ ] `/scan` 15 Hz 左右、frame `base_link`、ranges 有有限值；rviz 能看到与点云重合的扫描线
- [ ] slam_toolbox 输出 `/map`（resolution 0.05，max_laser_range 20），行走闭环无漂移爆图
- [ ] `ros2 param get /slam_toolbox solver_plugin`（或看日志）确认加载的是仓库 yaml 而不是默认配置
- [ ] nav2 生命周期节点全部 active；`ros2 topic hz /cmd_vel` 有输出（真机运动另行确认）
- [ ] 无 `Transform failure`、`package not found`、`Extrapolation` 类报错

## 9. 风险 & 备选

**风险**

1. 若该机器人固件不发布 `/utlidar/robot_odom`（bag 全都有，几乎不可能）→ 退用 `/lf/sportmodestate`（20 Hz，无四元数，需积分 yaw，质量差）。
2. 时钟域：机器人时间戳 ≈2025-04，主机 2026-11。本方案所有输出时间戳都用主机 `now()`（TF、`/odom`、`/scan`、`/trans_cloud`），因此干净；但如果以后用 bag 回放做离线测试，需要 `use_sim_time=true` + 时间重映射，另有一条工作流。
3. `global_costmap` 的 `static_layer` 订阅 `/map`（transient_local）。slam_toolbox 的 `/map` QoS 若不是 transient_local，静态层可能收不到——rviz 能看到 `/map` 就基本没问题；不行就临时把 static_layer 摘掉。
4. nav2 的 `map_server` 段引用了不存在的 turtlebot3 地图，但 `navigation_launch.py` 不启动 map_server，所以不报错；换“已知地图 + AMCL”模式时要另配。
5. 真机安全：vel 限制、吊起、急停（Ctrl-C 或 `/api/sport/request` StopMove）。

**备选方案 B（零侵入，不改仓库代码）**

新增一个 60 行小节点 `go2_pose_bridge`：订阅 `/utlidar/robot_odom`（Odometry）→
- 发布 `PoseStamped` 到 `/utlidar/robot_pose`（让仓库原封不动跑起来，TF、`/odom` 都由原 `go2_base` 产生）
- （可选）同时自己广播 `odom→base_link` TF。

优点：仓库零改动、可随时切回；缺点：多一个进程、`/odom.twist` 仍是 0（DWB 不依赖 twist，影响不大）、时钟仍要在 bridge 里改 stamp（同样得处理 568 天时差），且 §4.2/4.3 的感知质量问题依旧存在。
**取舍：推荐 A；B 只适合“先跑通、不改仓库”的临时验证。**

## 10. 需要你确认

1. 按 **方案 A** 改仓库（我直接改代码 + patch 记录），还是先要 **方案 B**（零侵入节点）？
2. 改动范围：只做 **P0**，还是 **P0+P1**（推荐，顺带把 nav2 速度和 `/scan` 质量修好）？
3. 是否由我执行：`apt install` 3 个包、改代码、`colcon build`、起节点做静态验证（不接真机动作）？
