import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('ascamera'), 'configurationfiles')

    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "ascamera_hp60cn",
            package='ascamera',
            executable='ascamera_node',
            respawn=True,
            output='both',
            parameters=[
                {"confiPath": config_path},
				{"pub_tfTree": True},
                {"color_pcl": False}
            ]
        ),
    ])
