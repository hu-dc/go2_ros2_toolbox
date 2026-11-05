# go2_ros2_toolbox 适配性检查报告（静态代码审查，未跑 bag）

检查对象：https://github.com/andy-zhuo-02/go2_ros2_toolbox.git（HEAD `cd11d74`，2025-07-20）
检查环境：本机 `ROS_DISTRO=humble` + `unitree_ros2`（cyclonedds，`CYCLONEDDS_URI` 绑 `ens37`）
数据依据：`/home/go2/go2_bags/` 下 22 个 bag 的 `metadata.yaml` 与 `.db3` 静态解码（未播放），以及本机已装 ROS 包清单

---

## 0. 结论（一句话）

**不适配。** 核心问题在话题订阅：代码的整条位姿链路都建立在 `/utlidar/robot_pose`（PoseStamped）上，
而你机器人的位姿话题是 `/utlidar/robot_odom`（`nav_msgs/msg/Odometry`，frame `odom`→`base_link`，约 242 Hz），
22 个 bag 里 **没有任何一条 `/utlidar/robot_pose` 消息**。这一条导致 TF `odom→base_link` 不发布、`/odom` 不发布、
`/scan` 因 TF 查询失败而不发布，SLAM 与 Nav2 全部空转。
此外仓库是 **ROS2 Foxy + Ubuntu 20.04** 时代产物，本机 Humble 下还缺 `slam_toolbox` / `nav2` 两个大包，参数名也有 Foxy→Humble 差异。

---

## 1. 本机机器人实际话题（bag 证据）

22 个 bag（20261102~20261105）录到的话题并集：

| 话题 | 类型 | 出现包数 | 频率（实测） | 备注 |
|---|---|---|---|---|
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 22/22 | 242.1 Hz | header.frame_id=`odom`，child_frame_id=`base_link`；msg 内 pose+twist 都有值 |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 22/22 | 249.6 Hz | frame `utlidar_imu`（L1 内置 IMU） |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 22/22 | 14.7 Hz | frame `utlidar_lidar`，6 field，point_step=32 |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 2/22 | 14.3 Hz | frame `odom`，x@0/y@4/z@8/intensity@12，point_step=16，width≈1031 |
| `/lf/lowstate`、`/lf/sportmodestate` | `unitree_go` | 22/22 | 20 Hz | |
| `/lowstate` | `unitree_go/msg/LowState` | 19/22 | ~500 Hz | 无 header.stamp |
| **`/utlidar/robot_pose`** | — | **0/22** | — | **不存在（无一条消息）** |
| `/tf`、`/odom`、`/cmd_vel` | — | 0/22 | — | 机器人不发 TF/odom，也不订阅 cmd_vel（走 `/api/sport/request`） |

> 说明：多数 bag 是显式话题列表录制（5~7 个话题），所以“某话题没出现”不能 100% 证明机器人不发；
> 但 `/utlidar/robot_odom` 22/22 稳定存在、`/utlidar/robot_pose` 一条都没有，且 unitree_ros2 文档也没有这个话题，
> 现有证据强烈指向：**你机器人的位姿源就是 `/utlidar/robot_odom`**，不是 `robot_pose`。
> 上机后用 `ros2 topic list | grep -i robot` 再确认一次即可。
>
> `cloud_deskewed` 只在 `20261103_140601/140730_walk_imu` 两个包出现，但这两个包里它是 **14.3 Hz 连续**的（1014/1038 帧），
> 说明机器人正常运行时确实在发这个话题（其他包是录制列表没包含它）。

---

## 2. 逐项话题核对（代码 vs 机器人）

| # | 代码中的话题/动作 | 代码位置 | 机器人实际 | 判定 |
|---|---|---|---|---|
| 1 | 订阅 `/utlidar/robot_pose`（`PoseStamped`） | `go2_core/src/go2_base.cpp:31-35` | 只有 `/utlidar/robot_odom`（`Odometry`） | ❌ **致命** |
| 2 | 订阅 `/utlidar/robot_pose`（`PoseStamped`） | `go2_perception/src/pointcloud_to_laserscan_node.cpp:85-88` | 同上 | ❌ 无效订阅（回调 `robotPoseCallback` 只做 `pose_stamped = *msg;`，从未被使用，见 `:153-157`） |
| 3 | 发布 `/odom`（`Odometry`） | `go2_base.cpp:45` + `:74-80` | Nav2 默认 `odom_topic: odom`，与仓库配置一致 | ✅（但 `twist` 恒为 0，见 §5） |
| 4 | 广播 TF `odom→base_link` | `go2_base.cpp:58-71` | 只在收到 `robot_pose` 时广播 → 实际不会广播 | ❌（由 #1 引起） |
| 5 | 订阅 `/cmd_vel`（`Twist`）→ `/api/sport/request` | `go2_base.cpp:38-42`、`:84-96` | Humble Nav2 由 `velocity_smoother` 发布最终 `/cmd_vel`；unitree_ros2 文档确认运动控制走 `/api/sport/request` | ✅ |
| 6 | 订阅 `/utlidar/cloud_deskewed` | `go2_perception/src/cloud_accumulation.cpp:78-80` | 存在，frame=`odom`，14.3 Hz，point_step=16 | ✅ |
| 7 | 发布 `cloud`（launch remap `/trans_cloud`） | `cloud_accumulation.cpp:75-76` + `launch/go2_pointcloud_process.launch.py:10` | 内部自洽 | ✅ |
| 8 | 订阅 `cloud_in`（remap `/trans_cloud`）、发布 `scan`→`/scan` | launch `:17-18`；`pointcloud_to_laserscan_node.cpp:90-103` | 内部自洽 | ✅ |
| 9 | `cloud_accumulation` 发布时 `frame_id="odom"`、`stamp=now()` | `cloud_accumulation.cpp:63-64, 87-88` | 与 deskewed 云 frame 吻合 | ✅（时序隐患见 §5） |
| 10 | `pointcloud_to_laserscan` 把云变换到 `base_link` | `:189-198`（`tf2_->transform`，timeout=0.01s） | 需要 TF `odom→base_link` → 不存在 → 每帧抛 `Transform failure` 并 return | ❌（由 #1/#4 引起，**无 /scan**） |
| 11 | `video_stream_node.py` 发布 `/camera/image_raw` | `go2_core/go2_core/video_stream_node.py`（GStreamer `multicast-iface=eth0`） | 仓库里没有任何节点消费它；本机没有 `eth0`（`ens33/ens37`），且机器人链路在 `ens37`（当前 `DOWN`，机器人未连） | ⚠ 不影响 SLAM/Nav2 |
| 12 | `go2_h264_repub` 发布 `/camera/image/raw` | `unitree_pkgs/go2_h264_repub/src/go2_h264_repub.cpp:9` | 与 `/camera/image_raw` **拼写不一致**；参数默认 `"etho0"`（typo）；全仓库没有任何 launch 引用它；且仓库里直接提交了 619 KB 预编译二进制 | ❌/未使用 |
| 13 | `slam_toolbox` 参数文件 `params_file` | `go2_slam/launch/go2_slamtoolbox.launch.py` | **Humble** 的 `online_async_launch.py` 参数名是 **`slam_params_file`** | ❌ 仓库 yaml 被静默忽略（退回 slam_toolbox 默认 yaml，默认值恰好接近，所以能跑但不生效） |
| 14 | Nav2 `params_file` | `go2_navigation/launch/go2_nav2.launch.py` | Humble `nav2_bringup/navigation_launch.py` 参数名就是 `params_file` | ✅ |
| 15 | `navigation_command_tcpbridge.py` 读 `go2_tcp_toolbox` 包 | `go2_navigation/**/navigation_command_tcpbridge.py:20-23` | 该包不存在，且 `go2_navigation/CMakeLists.txt` 只 install config/launch，不装 python 节点 | ❌ 不可用 |

---

## 3. 事故链（为什么整条链路都不动）

1. `go2_base` 等不到 `/utlidar/robot_pose` → 不广播 TF `odom→base_link`、不发布 `/odom`；
2. `pointcloud_to_laserscan` 收到 `/trans_cloud`（frame `odom`）后需要 TF `odom→base_link` 才能转到 `base_link`，
   TF 不存在 → 每帧 `Transform failure` 并 `return` → **不发布 `/scan`**；
3. `slam_toolbox` 无 `/scan` 输入 → 无地图、无 `map→odom`；
4. Nav2 无 `/odom`、无 TF、无 `map`、无 `/scan` → 控制器/规划/代价地图全部无法工作（即使 Nav2 装了）。

也就是说：**一个话题名（+消息类型）错了，工具箱的所有功能全断**，而不是“部分功能降级”。

---

## 4. 环境层面不适配（Foxy → Humble）

| 项 | 仓库假设（Foxy/20.04） | 本机（Humble） | 影响 |
|---|---|---|---|
| `slam_toolbox` | `ros-foxy-slam-toolbox` | **未安装**（`/opt/ros/humble/share` 无、全盘无） | `go2_slam` launch 在 `get_package_share_directory('slam_toolbox')` 处抛 `PackageNotFoundError`，`go2_startup` 启动即失败 |
| `nav2_bringup` / `navigation2` | `ros-foxy-nav2-bringup` | **未安装** | 同上，`go2_nav2` / `go2_startup` 立即失败 |
| Nav2 参数段名 | `recoveries_server` + `recovery_plugins`（Foxy） | Humble 是 `behavior_server` + `behavior_plugins` | 该段整体被忽略；Humble 默认 behavior 插件（spin/backup/drive_on_heading/assisted_teleop/wait）可用，故不致命 |
| BT 默认 XML 参数 | `default_bt_xml_filename`（Foxy） | Humble 叫 `default_nav_to_pose_bt_xml` | 参数被忽略 → 用 Humble 默认 `navigate_to_pose_w_replanning_and_recovery.xml`；该 XML 用到的节点库（`nav2_compute_path_to_pose_action_bt_node`、`nav2_follow_path_action_bt_node`、`nav2_clear_costmap_service_bt_node`、`nav2_goal_updated_condition_bt_node`、`nav2_recovery_node_bt_node`、`nav2_pipeline_sequence_bt_node`、`nav2_round_robin_node_bt_node`、`nav2_rate_controller_bt_node`、`nav2_spin/wait/back_up_action_bt_node`）都在仓库 `plugin_lib_names` 里 → **bt_navigator 大概率能跑** |
| Nav2 插件 id | 斜杠写法 | Humble 仍接受斜杠 id（`nav2_navfn_planner/NavfnPlanner`、`nav2_behaviors/Spin`、`dwb_core::DWBLocalPlanner` 等） | ✅ 兼容，不是失效点 |
| `controller_server` 检查器参数 | `progress_checker_plugin`（单数）、`goal_checker_plugin`（单数） | Humble：`progress_checker_plugin` **仍是单数**✅；`goal_checker_plugin` 改成了 **`goal_checker_plugins`**（复数）→ 这行被忽略，但默认 id 正好是 `goal_checker`，仍会命中仓库的 `goal_checker:` 段 → 参数实际生效 | 基本无影响 |
| `velocity_smoother` 参数段 | 仓库未提供 | Humble 默认 `max_velocity=[0.50, 0, 2.50]`（见 `nav2_velocity_smoother` 源码） | DWB 的 `max_vel_x: 3.0` 实际被限到 0.5 m/s；想恢复速度需补 `velocity_smoother` 段 |
| `amcl` / `map_server` 段 | 有，但 `map_server.yaml_filename: "turtlebot3_world.yaml"` 是 turtlebot3 遗留 | `navigation_launch.py` 根本不启动 amcl/map_server | 该段无效（建图用 slam_toolbox 出图，走 static layer），不致命；但若改用 bringup 会因地图文件不存在而失败 |
| 自带 `unitree_api`/`unitree_go` | 与本机已装 unitree_ros2 重名 | 消息定义完全一致（仅 `Go2FrontVideoData.msg` 差一个行尾空格；安装版多 `MotorCmds/MotorStates`） | 同 workspace 构建会覆盖安装版，类型兼容，不致命（属冗余） |

> 安装命令（Humble）：`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox`

---

## 5. 非话题但会影响可用性的问题

1. **时钟域（隐性坑，很重要）**：机器人话题时间戳是**机身时钟**（实测 `/utlidar/robot_odom` `sec=0x67fe123a` ≈ 2025-04-15，与主机相差 ~568 天）。
   任何“按消息 stamp 查 TF”的节点（`pointcloud_to_laserscan` 的 `tf2_->transform(..., cloud.stamp)`、slam_toolbox、Nav2 代价地图）都会因时钟域不同而失败。
   仓库目前**碰巧绕开**了：`cloud_accumulation` 发布时把 stamp 改成 `now()`（`:87-88`），`go2_base` 的 TF/odom stamp 也用 `this->now()`（`:58, :75`）。
   ⚠ 如果为了“直连点云”把 `/utlidar/cloud_deskewed` 直接 remap 给 `pointcloud_to_laserscan`（跳过 accumulation），
   stamp 会变回机器人时钟，TF 查询立刻全失败 —— 所以“重打时间戳”这一步不能省。
2. **`cloud_accumulation` 的 2 秒拼接**：把最近 **30 帧**（14.3 Hz ≈ 2.1 s）点云拼成一帧，只用**发布时刻**一个 stamp
   （`:18, :24-33, :87`）。转向时点云内容横跨 2 s，slam_toolbox 会得到“拖影”激光；而且 50 Hz 发布 + 每帧约 30×1031×16B≈500 KB → DDS 约 25 MB/s。
   建议：N 降到 1（透传 + 重打 stamp），或直接改成一个“re-stamp 转发”节点。
3. **`pointcloud_to_laserscan` 订阅线程 bug**：`subscriptionListenerThreadLoop()` 里原本“有订阅者才订阅”的判断被整段注释
   （`:126-141`），改成每 100 ms 无条件 `sub_.subscribe(this, "cloud_in", qos)`（`:143-146`）→ 订阅被反复重建，可能丢帧、白耗 CPU。
4. **`/odom.twist` 恒为 0**：`go2_base.cpp:79-80` 完全没填 twist；而你机器人 `/utlidar/robot_odom` 的 twist 是有值的
   （实测静止时仍有 ~0.02 rad/s 量级）。Nav2 的 DWB / velocity_smoother 用 twist 做速度反馈，建议透传。
5. **激光参数自相矛盾**：`scan_time` 设 0.25（4 Hz），实际 `/scan` 以 50 Hz 发布；`angle_increment 0.0087` 与 ±3.14 匹配（722 线）OK；
   `min_height -0.1 / max_height 1.0`：由于 `robot_odom` 的 pose.z=0.32（`base_link` 离地约 0.32 m），地面点落在 z≈-0.32 被 `min_height` 滤掉 ✅，最大 1.0 m 也会把高处障碍裁掉。
6. **包依赖声明不全**：`go2_core/package.xml` 少了 `nav_msgs`、`tf2_ros`、`tf2`（CMake 里有 `find_package`）；`go2_perception/package.xml` 几乎是空壳（没有 `rclcpp/sensor_msgs/tf2_ros/message_filters/laser_geometry` 等），rosdep 会漏装。构建在本机可用（包都在），但换机器会踩坑。
7. **文档不一致**：`README.md:113-114`、`README_zh.md:120-121` 把 `/utlidar/robot_pose`、`/utlidar/cloud_deskewed` 列在 **Publishers** 表里，
   但它们其实是机器人自己发的话题；工具箱实际发布的是 `/odom`、TF、`/trans_cloud`。代码与文档都与你的机器人现状对不上。
8. **摄像头链路**：`video_stream_node.py` 用 `multicast-iface=eth0`（本机没有 eth0，机器人链路对应 `ens37`），且 `startup` 默认 `video_enable:=true`
   → 启动时会报 GStreamer 错误（不影响 SLAM/Nav2，但很吵）；`go2_h264_repub` 的 `/camera/image/raw` 拼写不一致且无人消费，建议别用。

---

## 6. 修复建议（最小改动即可跑通）

### 方案 A（推荐）：把 `go2_base` 的位姿源换成 `/utlidar/robot_odom`

`go2_core/src/go2_base.cpp`：

```cpp
#include <nav_msgs/msg/odometry.hpp>

// 替换 :31-35
odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
  "/utlidar/robot_odom", 10,
  [this](const nav_msgs::msg::Odometry::SharedPtr msg) { handleOdom(msg); });

// 新增（替换 handlePose）
void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();                 // 保持主机时钟（关键）
  t.header.frame_id = "odom";
  t.child_frame_id  = "base_link";
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.rotation      = msg->pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = t.header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id  = "base_link";
  odom_msg.pose  = msg->pose;
  odom_msg.twist = msg->twist;                 // 顺便修掉 twist 恒 0
  odom_pub_->publish(odom_msg);
}
```

### 方案 B（零改仓库）：加一个 20 行适配节点

订阅 `/utlidar/robot_odom`，转成 `PoseStamped` 发到 `/utlidar/robot_pose`（frame `odom`）。
这样 `go2_base`、`pointcloud_to_laserscan` 都能原样工作；缺点是 `/odom.twist` 仍然为 0。

### 其余必改/建议

1. 装包：`ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox`；
2. `go2_slam/launch/go2_slamtoolbox.launch.py`：`'params_file'` → `'slam_params_file'`；
3. `nav2_params.yaml`：`recoveries_server`→`behavior_server`（或直接删掉，用 Humble 默认）、`default_bt_xml_filename`→`default_nav_to_pose_bt_xml`（或删）、删/换 `map_server` 的 `turtlebot3_world.yaml`、按需补 `velocity_smoother` 段；
4. `cloud_accumulation.cpp`：`30` 改 `1`（或改成“透传 + `stamp=now()`”）；`scan_time` 按实际发布频率改（≈0.02~0.07）；
5. `pointcloud_to_laserscan_node.cpp`：恢复 `:126-141` 的订阅守卫（或只 `subscribe` 一次），删掉 `:85-88` 的死订阅；
6. `go2_startup.launch.py`：不用摄像头时 `video_enable:=false`；要用就把 `multicast-iface` 改成实际接机器人的网卡（本机 `ens37`）；
7. `go2_navigation` 的 python 节点：要么补 `go2_tcp_toolbox` 与安装规则，要么删掉。

---

## 7. 上机验证清单（不用 bag，只看代码之外的最小确认）

```bash
source /home/go2/unitree_ros2/setup.sh          # RMW=rmw_cyclonedds_cpp, iface=ens37
ip -br link                                      # ens37 必须 UP（当前 DOWN，机器人未连）
ros2 topic list | grep -E "utlidar|robot|cmd_vel|tf"
ros2 topic info /utlidar/robot_odom -v           # 确认类型 nav_msgs/msg/Odometry
ros2 topic hz /utlidar/robot_odom                # 预期 ~242 Hz
ros2 topic hz /utlidar/cloud_deskewed            # 预期 ~14.3 Hz
```

改完代码后：

```bash
ros2 topic echo /odom --once                      # pose 有值、twist 不再全 0
ros2 run tf2_ros tf2_echo odom base_link          # TF 出现，z≈0.32
ros2 topic hz /scan                               # /scan 出现
ros2 topic hz /map                                # slam_toolbox 出图
```

---

## 8. 附：本次检查用到的关键 bag 统计（20261103_140601_walk_imu，70.7 s）

```
/utlidar/robot_odom   17110 msgs  242.09 Hz
/utlidar/imu          17639 msgs  249.57 Hz
/utlidar/cloud         1038 msgs   14.69 Hz
/utlidar/cloud_deskewed 1014 msgs  14.34 Hz
/lowstate             35256 msgs  498.82 Hz
/lf/lowstate           1414 msgs   20.00 Hz
/lf/sportmodestate     1413 msgs   20.00 Hz
```

`/utlidar/cloud_deskewed` 解出的字段：`height=1, width=1031, point_step=16, row_step=16496`，
fields = `x@0, y@4, z@8, intensity@12`，`frame_id="odom"` —— 与 `cloud_accumulation.cpp`
里“只取 `fields[0..2].offset` 的 x/y/z、整点拷贝 `point_step`”的写法一致（能工作，但依赖字段顺序）。
