#!/usr/bin/env python3
"""启动 MoveIt move_group 节点（连接 Gazebo 仿真中的 arm_controller 执行）"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # 所有 MoveIt 配置文件位于包根 config/ 目录（fishbot.srdf / kinematics.yaml /
    # joint_limits.yaml / moveit_controllers.yaml / ompl_planning.yaml）
    moveit_config = (
        MoveItConfigsBuilder("fishbot", package_name="fishbot_description")
        .robot_description(file_path="urdf/fishbot/fishbot.urdf.xacro")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
        ]
    )
