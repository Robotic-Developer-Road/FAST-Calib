import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_fast_calib = get_package_share_directory('fast_calib')
    
    # Configuration files
    config_file = os.path.join(pkg_fast_calib, 'config', 'qr_params.yaml')
    rviz_config = os.path.join(pkg_fast_calib, 'rviz_cfg', 'fast_livo2.rviz')

    # Launch arguments
    use_rviz = LaunchConfiguration('rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        
        Node(
            package='fast_calib',
            executable='fast_calib',
            name='fast_calib',
            output='screen',
            parameters=[config_file]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=None # Handle rviz argument if needed
        )
    ])
