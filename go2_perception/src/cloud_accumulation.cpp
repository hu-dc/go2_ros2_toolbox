#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


sensor_msgs::msg::PointCloud2 accumulated_cloud;
std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;

void cloudCallback(
sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
    // 直接将新的点云添加到 clouds 中
    clouds.push_back(cloud_msg);

    // 累积最近 5 帧（约 0.35 秒）拼接成一帧发布，让点云更密、RViz 里更容易看到。
    // 每帧点云都已是 odom 坐标系（/utlidar/cloud_deskewed 的 frame_id=odom），
    // 在 odom 中拼接是自洽的；原实现保留 30 帧（约 2 秒），机器人走动时拼接跨度太大。
    if (clouds.size() > 5)
    {
        clouds.erase(clouds.begin());
    }

    // 创建一个新的点云来存储合并后的点云
    auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    *merged_cloud = *clouds[0];

    // 遍历 clouds 并将所有的点云合并到 merged_cloud 中
    for (size_t it = 1; it < clouds.size(); it++)
    {
        merged_cloud->width += clouds[it]->width;
        merged_cloud->row_step += clouds[it]->row_step;
        merged_cloud->data.insert(merged_cloud->data.end(), clouds[it]->data.begin(), clouds[it]->data.end());
    }

    // 注意：这里不再做 z∈[0.2,1.0] 的高度裁剪。
    // /utlidar/cloud_deskewed 已是 odom 坐标系（地面 z≈0），按 0.2~1.0 裁剪会把
    // 绝大部分点（地面/低处墙面）删掉，RViz 里几乎看不到点云。
    // 2D 扫描所需的高度过滤由 pointcloud_to_laserscan 在 base_link 系完成。
    accumulated_cloud = *merged_cloud;
    accumulated_cloud.header.frame_id = "odom";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.reliable();

  auto node = std::make_shared<rclcpp::Node>("cloud_accumulations");
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos);

  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud_deskewed", rclcpp::SensorDataQoS(),
    std::bind(&cloudCallback, std::placeholders::_1));


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // 输入约 14.3Hz，单帧透传时按 15Hz 发布即可（原 50Hz 只是重复发布同一帧）
  rclcpp::Rate rate(15.0);
  while (rclcpp::ok()) {
    executor.spin_some();

    // 首帧到达前不要发布空点云
    if (!clouds.empty()) {
      accumulated_cloud.header.stamp = node->get_clock()->now();
      pub->publish(accumulated_cloud);
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
