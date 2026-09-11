import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('ascamera'), 'configurationfiles')

    ld = LaunchDescription()
    ascamera_node = Node(
        namespace= "ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": 2},
            {"usb_path": "null"},
            {"confiPath": config_path},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps":10},
        ],
        remappings=[]
    )

    # ascamera_node2 = Node(
    #     namespace= "ascamera_hp60c_2",
    #     package='ascamera',
    #     executable='ascamera_node',
    #     respawn=True,
    #     output='both',
    #     parameters=[
    #         {"usb_bus_no": 3},    # set your usb_bus_no
    #         {"usb_path": "2.2"},  # set your usb_path
    #         {"confiPath": "./ascamera/configurationfiles"},
    #         {"color_pcl": False},
    #         {"depth_width": 640},
    #         {"depth_height": 480},
    #         {"rgb_width": 640},
    #         {"rgb_height": 480},
    #         {"fps": 15},
    #     ],
    #     remappings=[]
    # )

    ld.add_action(ascamera_node)
    # ld.add_action(ascamera_node2)

    return ld

