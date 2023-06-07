from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from os.path import join

def generate_launch_description():
    params_file = join(
        get_package_share_directory("eufs_tracks"),
        "params",
        "headless_track_generator.yaml"
    )
    return LaunchDescription([
        Node(
            name='headless_track_generator',
            package='eufs_tracks',
            executable='headless_track_generator',
            parameters=[params_file],
        ),
    ])
