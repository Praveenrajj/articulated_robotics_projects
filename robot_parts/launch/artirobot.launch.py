import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg = 'robot_parts'
    pkg_share = get_package_share_directory(pkg)

    xacro_file = os.path.join(pkg_share, 'description', 'two_wheeled_robot.urdf.xacro')
    world = os.path.join(pkg_share, 'worlds', 'cardboard_obstacle.world')
    controllers_yaml = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    # 1) RSP: expand xacro into robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # 2) Gazebo (world optional)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # 3) Spawn the model in Gazebo (reads robot_description)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # 4) Controller spawners AFTER the model is inserted
    spawner_jsb = Node(
        package='controller_manager', executable='spawner.py', output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--param-file', controllers_yaml
        ]
    )

    spawner_diff = Node(
        package='controller_manager', executable='spawner.py', output='screen',
        arguments=[
            'diff_cont',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--param-file', controllers_yaml
        ]
    )

    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[spawner_jsb, spawner_diff])
    )

    return LaunchDescription([rsp, gazebo, spawn, after_spawn])
