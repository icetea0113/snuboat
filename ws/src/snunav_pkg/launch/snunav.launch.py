import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('snunav_pkg'),
        'config',
        'params.yaml'
        )
        
    mission_params = {}
    if 'mission_director' in config:
        mission_params = config.get('mission_director', {}).get('ros__parameters', {})
    if 'controller' in config:
        controller_params = config.get('controller', {}).get('ros__parameters', {})

    return LaunchDescription([
        Node(
            package='snunav_pkg',
            executable='mission_director',    # 노드 실행 파일 이름
            name='mission_director_node',     #
            output='screen',
            parameters=[mission_params]
        ),
        Node(
            package='snunav_pkg',
            executable='controller',    # 노드 실행 파일 이름
            name='controller_node',     #
            output='screen',
            parameters=[controller_params]
        ),
    ])