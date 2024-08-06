from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    # Declare the argument for the RViz configuration file path

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=EnvironmentVariable(
            'RVIZ_CONFIG',
            default_value='/home/asaf/visual_odom_project/src/project_bringup/rviz/rviz_config.rviz'
        ),
        description='Path to the RViz configuration file'
    ) 

    # Define the main node
    main_node = Node(
        package='my_project_pkg',
        executable='optical_flow_run',
        name='main_node',
        output='screen'
    )

    # Define the RViz node with the configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # Define the camera node


    # Create the launch description and populate it with nodes and arguments
    ld = LaunchDescription([
        rviz_config_arg,
        main_node,
        rviz_node,
    ])

    return ld
