import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from datetime import datetime

maneuver_mode_map = {
    '1': 'FREE_RUNNING',
    '2': 'DOCKING',
    '3': 'DP',
    '4': 'NAVIGATION'
}

# 상위 모드별 세부 모드 매핑
sub_maneuver_mode_map = {
    '1': {  # Free running
        '0': 'SPEED_MAPPING',
        '1': 'TURNING',
        '2': 'ZIGZAG',
        '3': 'PIVOT_TURN',
        '4': 'CRABBING',
        '5': 'PULL_OUT',
        '6': 'SPIRAL'
    },
    '2': {  # Docking
        '0': 'HEUR_ENTER',
        '1': 'DRL_ENTER',
        '2': 'HEUR_MULTI',
        '3': 'DRL_MULTI'
    },
    '3': {  # DP (To-be-defined)
        '0': 'TBD_0',
        '1': 'TBD_1'
    },
    '4': {  # Navigation (To-be-defined)
        '0': 'TBD_0',
        '1': 'TBD_1'
    }
}

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
    
    # 2. 코드 추출
    man_code = mission_params.get('maneuver_mode', '-1')
    sub_code = mission_params.get('sub_maneuver_mode', '-1')

    # 3. 코드 → 이름 변환
    man_name  = maneuver_mode_map.get(man_code, str(man_code))
    sub_name  = sub_maneuver_mode_map.get(man_code, {}).get(sub_code, str(sub_code))

    # 4. bag 이름 생성
    ts = datetime.now().strftime('%m%d_%H%M%S')
    bag_name = os.path.expanduser(f"~/snuboat/ws/src/{man_name}_{sub_name}_{ts}")

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
    
    ld.add_action(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '/sensor', '/ctrl_cmd_sils',
                 '-o', bag_name],
            output='screen'
        )
    )
    
    return ld