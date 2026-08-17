import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import shlex

def generate_launch_description():
    # 获取功能包的 share 路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path,'urdf','fishbot/fishbot.urdf.xacro') 
    # default_rviz_path = os.path.join(urdf_package_path,'config','display_robot_model.rviz')
    # default_gazebo_world_path = os.path.join(urdf_package_path,'world','custom_room.world')
    # default_gazebo_world_path = os.path.join(urdf_package_path,'world','narrow_corridor.world')
    # default_gazebo_world_path = os.path.join(urdf_package_path,'world','U_shaped_obstacle.world')
    # default_gazebo_world_path = os.path.join(urdf_package_path,'world','bigger_room.world')
    default_gazebo_world_path = os.path.join(urdf_package_path,'world','bigger_room_complex.world')
    # default_gazebo_world_path = os.path.join(urdf_package_path,'world','bigger_room_without_person.world')

    # 声明一个 urdf 目录的默认参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='加载的模型文件路径'
    )
    # world 参数可覆盖（用于测试/切换场景）
    action_declare_world_arg = launch.actions.DeclareLaunchArgument(
        name='world',default_value=str(default_gazebo_world_path),description='Gazebo 世界文件路径'
    )
    # 通过文件路径，获取内容，并转换成参数值对象，以供传入 robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',launch.substitutions.LaunchConfiguration('world')),('verbose','true')]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','fishbot']
    )

    # 等待 Gazebo 世界加载完成后再 spawn（并发启动时过早 spawn 会导致 gzserver 崩溃）
    action_delayed_spawn = launch.actions.TimerAction(
        period=10.0,
        actions=[action_spawn_entity]
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    action_load_arm_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller arm_controller --set-state active'.split(' '),
        output='screen'
    )

    action_load_effort_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_effort_controller --set-state active'.split(' '),
        output='screen'
    )

    action_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    # Panda 初始姿态设为 ready（franka 官方初始，避免折叠零位自碰撞影响 MoveIt 规划）
    action_set_ready_pose = launch.actions.ExecuteProcess(
        cmd=shlex.split('ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f '
             '"{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], '
             'points: [{positions: [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854], time_from_start: {sec: 2}}]}}"'),
        output='screen'
    )
    
    # action_rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', default_rviz_path]
    # )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_world_arg,
        action_robot_state_publisher,
        # action_joint_state_publisher,
        action_launch_gazebo,
        action_delayed_spawn,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_arm_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_arm_controller,
                on_exit=[action_diff_drive_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_diff_drive_controller,
                on_exit=[action_set_ready_pose],
            )
        ),

        # action_rviz_node,
    ])