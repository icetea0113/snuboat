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
    
    # SLAM 모드 (sensor_mode == '1'): Ouster + KISS-ICP
    if sensor_mode == '1':
        # Ouster 센서 launch
        ouster_launch = IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ouster_ros'),
                    'launch',
                    'sensor.launch.xml'
                )
            ),
            launch_arguments={
                'sensor_hostname': '192.168.1.3',
                'viz': 'false'  # rviz 비활성화 (필요시 수정)
            }.items()
        )
        ld.add_action(ouster_launch)
        
        # KISS-ICP odometry launch
        kiss_icp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('kiss_icp'),
                    'launch',
                    'odometry.launch.py'
                )
            ),
            launch_arguments={
                'topic': '/ouster/points',
                'lidar_odom_frame': 'odom',
                'visualize': 'false'
            }.items()
        )
        ld.add_action(kiss_icp_launch)
    
    # GPS-RTK 모드 (sensor_mode == '2'): Microstrain IMU
    elif sensor_mode == '2':
        microstrain_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('microstrain_inertial_driver'),
                    'launch',
                    'microstrain_launch.py'
                )
            )
        )
        ld.add_action(microstrain_launch)
    
    return ld