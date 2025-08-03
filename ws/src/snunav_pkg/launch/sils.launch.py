import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # Config 파일 읽기
    config_path = os.path.join(
        get_package_share_directory('snunav_pkg'),
        'config',
        'params.yaml'
    )
    
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    # 파라미터 추출
    mission_params = {}
    controller_params = {}
    
    if 'mission_director' in config:
        mission_params = config.get('mission_director', {}).get('ros__parameters', {})
    if 'controller' in config:
        controller_params = config.get('controller', {}).get('ros__parameters', {})
    
    # sensor_mode 값 가져오기
    sensor_mode = mission_params.get('sensor_mode', '0')
    
    # 기존 노드들
    ld.add_action(Node(
        package='snunav_pkg',
        executable='mission_director',
        name='mission_director_node',
        output='screen',
        parameters=[mission_params]
    ))
    
    ld.add_action(Node(
        package='snunav_pkg',
        executable='controller',
        name='controller_node',
        output='screen',
        parameters=[controller_params]
    ))
    
    ld.add_action(Node(
        package='snunav_pkg',
        executable='navigation',
        name='navigation_node',  # controller_node에서 수정
        output='screen',
    ))
    
    ld.add_action(Node(
        package='snunav_pkg',
        executable='sils',
        name='sils_node',
        output='screen',
    ))
    
    return ld