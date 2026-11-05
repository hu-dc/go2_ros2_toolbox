"""实时可视化：位姿桥 + 点云转激光 + RViz，不依赖 slam_toolbox / nav2。

用途：接上机器人后，一条命令就能在 RViz 里实时看到：
  /odom（里程计箭头）、TF(odom->base_link)、/trans_cloud（点云，frame odom）、/scan（激光）、/map（有 SLAM 时）
启动：
  ros2 launch go2_core go2_live_view.launch.py
可选参数：
  rviz:=false            只跑节点不开 RViz
  video_enable:=true     同时开相机（需先改 video_stream_node.py 里的组播网卡）
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    go2_core_dir = get_package_share_directory('go2_core')
    go2_perception_dir = get_package_share_directory('go2_perception')

    rviz_config = os.path.join(go2_core_dir, 'config', 'default.rviz')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true', description='是否打开 RViz'))
    ld.add_action(DeclareLaunchArgument(
        'video_enable', default_value='false', description='是否启用相机'))
    ld.add_action(DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw', description='相机图像话题'))

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_core_dir, 'launch', 'go2_base.launch.py')),
        launch_arguments={
            'video_enable': LaunchConfiguration('video_enable'),
            'image_topic': LaunchConfiguration('image_topic'),
        }.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_perception_dir, 'launch', 'go2_pointcloud_process.launch.py')),
    )

    # 不显式指定 name：否则 launch 会生成 `-r __node:=rviz2`，把 RViz 进程内
    # 所有子节点都改名成 rviz2，导致 “Publisher already registered for provided
    # node name” 一类重复节点名警告。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    ld.add_action(base_launch)
    ld.add_action(perception_launch)
    ld.add_action(rviz_node)

    return ld
