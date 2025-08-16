import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_name="robot_parts"
    
    xacro_file=os.path.join(get_package_share_directory(pkg_name),"description","two_wheeled_robot.xacro")
    robot_description=xacro.process_file(xacro_file).toxml()

    node_rsp= Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description':robot_description,'use_sim_time':True}]
    )

 

    jsp= Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )


    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    ),
    launch_arguments={
        'world': os.path.join(get_package_share_directory(pkg_name), 'worlds', 'cardboard_obstacle.world')
    }.items()
)
    

    spawn_entity= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "two_wheeled_robot",
            "-topic", "robot_description"
        ],
        output="screen"
    )


    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[]
    )

    return LaunchDescription([
        node_rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        # jsp
    ])