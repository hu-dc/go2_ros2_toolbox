#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <string>
#include "rmw/rmw.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <array>

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

// 读取环境变量并把多行/多余空白压成一行（CYCLONEDDS_URI 是多行 XML）
static std::string envOrUnset(const char* key) {
    const char* v = std::getenv(key);
    if (!v) return "<unset>";
    std::string out;
    bool space = false;
    for (const char* p = v; *p; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        if (c == '\n' || c == '\r' || c == '\t' || c == ' ') { space = true; continue; }
        if (space && !out.empty()) out += ' ';
        space = false;
        out += static_cast<char>(c);
    }
    return out;
}

class Go2Base : public rclcpp::Node {
public:
    Go2Base() : Node("go2_base") {
        // 初始化运动客户端
        sport_client_ = std::make_shared<SportClient>();
        
        // 创建TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 订阅机器人里程计（Go2 实际话题为 /utlidar/robot_odom，nav_msgs/Odometry，约 242Hz）
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                handleOdom(msg);
            });
            
        // 订阅速度命令
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handleVelocity(msg);
            });
        
        // 创建里程计发布者
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // 运动指令发布
        sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // 安全看门狗：上位机/nav2 异常导致 /cmd_vel 断流时自动 StopMove，
        // 防止机器人按最后一帧速度一直运动（真机安全第一）
        last_cmd_time_ = this->now();
        safety_timer_ = create_wall_timer(100ms, [this]() { checkCmdVelTimeout(); });

        // 启动时打印实际生效的 DDS 环境，方便一眼看出“收不到数据”是不是环境问题
        RCLCPP_INFO(get_logger(),
                    "DDS 环境: rmw=%s; RMW_IMPLEMENTATION=%s; ROS_LOCALHOST_ONLY=%s; CYCLONEDDS_URI=%s",
                    rmw_get_implementation_identifier(), envOrUnset("RMW_IMPLEMENTATION").c_str(),
                    envOrUnset("ROS_LOCALHOST_ONLY").c_str(), envOrUnset("CYCLONEDDS_URI").c_str());

        // 数据看门狗：收不到 /utlidar/robot_odom 时明确报错（最常见原因是没 source
        // unitree_ros2 的 setup.sh → RMW 不是 rmw_cyclonedds_cpp / CYCLONEDDS_URI 没绑 ens37）
        odom_watch_timer_ = create_wall_timer(1s, [this]() { checkOdomTimeout(); });
    }

private:
    // 处理里程计消息
    void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        got_odom_ = true;
        last_odom_time_ = this->now();
        // 创建TF变换
        geometry_msgs::msg::TransformStamped t;
        
        // 设置变换的时间戳和坐标系
        // 注意：机器人自身时钟可能与主机不同步（bag 中相差数百天），
        // 这里统一使用主机时间 now()，保证 TF/scan/odom 处在同一时钟域
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        // 设置位置
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        
        // 设置旋转
        t.transform.rotation = msg->pose.pose.orientation;
        
        // 发布变换
        tf_broadcaster_->sendTransform(t);

        // 构造并发布Odometry消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = t.header;
        odom_msg.child_frame_id = t.child_frame_id;
        odom_msg.pose = msg->pose;    // 位置、姿态（含协方差）直接透传
        odom_msg.twist = msg->twist;  // 速度直接透传（原实现恒为 0）
        odom_pub_->publish(odom_msg);
    }

    // 处理速度命令
    void handleVelocity(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->now();
        // 直接转发速度命令
        unitree_api::msg::Request req;
        sport_client_->Move(req, 
                          msg->linear.x,
                          msg->linear.y,
                          msg->angular.z);
        sport_pub_->publish(req);
        
        if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) {
            moving_ = false;
            stopRobot();
        } else {
            moving_ = true;
        }
    }

    // /cmd_vel 断流看门狗：0.5s 没有新速度指令就停
    void checkCmdVelTimeout() {
        if (!moving_) {
            return;
        }
        if ((this->now() - last_cmd_time_).seconds() > 0.5) {
            moving_ = false;
            RCLCPP_WARN(get_logger(), "No /cmd_vel for 0.5s, sending StopMove");
            stopRobot();
        }
    }

    // /utlidar/robot_odom 看门狗：3s 没有数据就周期性提示，避免“RViz 里什么都没有但看不出原因”
    void checkOdomTimeout() {
        const auto now = this->now();
        if (got_odom_ && (now - last_odom_time_).seconds() <= 3.0) {
            return;
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "%s /utlidar/robot_odom; 检查: 1) source /home/go2/unitree_ros2/setup.sh "
            "（只 source cyclonedds_ws/install/setup.bash 不会设置 RMW/CYCLONEDDS_URI）"
            "2) echo $RMW_IMPLEMENTATION 应为 rmw_cyclonedds_cpp "
            "3) CYCLONEDDS_URI 绑定的网卡(ens37)是否 UP 且机器人网段 192.168.123.x",
            got_odom_ ? "3s 未收到新的" : "等待");
    }

    // 停止机器人
    void stopRobot() {
        unitree_api::msg::Request req;
        sport_client_->StopMove(req);
        sport_pub_->publish(req);
    }

    // 成员变量
    std::shared_ptr<SportClient> sport_client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // ROS接口
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;

    // 安全看门狗状态
    rclcpp::TimerBase::SharedPtr safety_timer_;
    rclcpp::Time last_cmd_time_;
    bool moving_{false};

    // 数据看门狗状态
    rclcpp::TimerBase::SharedPtr odom_watch_timer_;
    rclcpp::Time last_odom_time_;
    bool got_odom_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2Base>());
    rclcpp::shutdown();
    return 0;
} 