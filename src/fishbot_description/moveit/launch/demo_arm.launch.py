#!/usr/bin/env python3
"""MoveIt + RViz 演示（机械臂运动规划可视化，可连接 Gazebo 仿真执行）"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    moveit_config = (
        MoveItConfigsBuilder("fishbot", package_name="fishbot_description")
        .robot_description(file_path="urdf/fishbot/fishbot.urdf.xacro")
        .robot_description_semantic(file_path="moveit/config/fishbot.srdf")
        .trajectory_execution(file_path="moveit/config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="moveit/config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # RViz 使用 MoveIt 自带的运动规划面板配置
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_ros_visualization"),
        "rviz",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # 静态 TF：base_footprint -> base_link（MoveIt 演示时需要，Gazebo 中由 robot_state_publisher 提供）
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_footprint", "base_link"],
    )

    return LaunchDescription(
        declared_arguments + [move_group_node, rviz_node, static_tf]
    )
