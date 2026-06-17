import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vrpn_mocap_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vrpn_mocap'),
                'launch',
                'client.launch.yaml',
            )
        ),
        launch_arguments={
            'server': '192.168.250.25',
            'port': '3883',
        }.items(),
    )

    rotator_node = ExecuteProcess(
        cmd=[sys.executable,
             os.path.join(os.path.dirname(__file__), 'mocap_rotate_node.py')],
        output='screen',
    )

    return LaunchDescription([
        vrpn_mocap_launch,
        rotator_node,
    ])
