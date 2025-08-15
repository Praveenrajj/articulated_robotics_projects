import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
        rviz_node,
        jsp
    ])