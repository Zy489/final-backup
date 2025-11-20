from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robot_control_architecture_pkg'

    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='robot_quiz_demo_hardcode',
            name='quiz_demo_hardcode',
            output='screen',
            parameters=[params_file],
            # 如果七号机的速度话题不是 /cmd_vel，可以在这里 remap：
            # remappings=[('cmd_vel', '/robot7/cmd_vel')],
        ),
        Node(
            package=pkg_name,
            executable='simple_tts_node',
            name='simple_tts',
            output='screen',
        ),
    ])
