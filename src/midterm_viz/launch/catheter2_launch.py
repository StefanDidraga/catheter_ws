import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('midterm_viz')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'catheter_viz.rviz')
    
    micro_ros_agent_1 = ExecuteProcess(
        cmd=[
            'docker', 'run', '--init', '--rm', 
            '-v', '/dev:/dev', '--privileged', '--net=host', 
            'microros/micro-ros-agent:jazzy', 
            'serial', '--dev', '/dev/ttyUSB0', '-v6'
        ],
        output='screen'
    )
    return LaunchDescription([
        micro_ros_agent_1,
        Node(
            package='midterm_viz',
            namespace='mock_catheter_node',
            executable='mock_catheter_node',
            name='mock_catheter_node',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])