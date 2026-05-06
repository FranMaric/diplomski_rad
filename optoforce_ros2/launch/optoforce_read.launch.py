import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # optoforce_params = os.path.join(
    #     get_package_share_directory('optoforce_wrapper'),
    #     'config',
    #     'optoforce_params.yaml'
    #     )
        
    optoforce_node = Node(
        package = 'optoforce_wrapper',
        executable = 'optoforce_node'
    )

    optoforce_filter_node = Node(
        package = 'optoforce_wrapper',
        executable = 'force_low_pass_filter'
    )

    return LaunchDescription([
        optoforce_node,
        optoforce_filter_node
    ])