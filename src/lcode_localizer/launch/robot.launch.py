import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    pkg_localizer = get_package_share_directory('lcode_localizer')
    pkg_description = get_package_share_directory('yahboomcar_description')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # 파일 경로
    nav2_params = os.path.join(pkg_localizer, 'params', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_localizer, 'params', 'lcode_map.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'Raspbot-V2.urdf')

    # URDF 데이터 직접 읽기
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # 2. Robot State Publisher (use_sim_time: False)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False, 
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
            'frame_prefix': ''
        }]
    )

    # 3. [추가] 실제 카메라 노드 
    camera_node = Node(
        package='camera_node',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. [추가] 실제 로봇 모터/오도메트리 드라이버 노드
    # yahboomcar_bringup 등 프로젝트에 맞춰 실행파일명을 조정하세요.
    robot_driver_node = Node(
        package='driver_node',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 5. L-Code Localizer (use_sim_time: False)
    localizer_node = Node(
        package='lcode_localizer',
        executable='lcode_localizer_node',
        name='lcode_localizer',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 6. Nav2 Bringup (use_sim_time: False 반영)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'yaml_filename': map_yaml,
            'use_sim_time': 'False',  # 중요!
            'params_file': nav2_params,
            'autostart': 'True',
            'use_lifecycle_mgr': 'True'
        }.items()
    )

    # 7. App Bridge
    app_bridge_node = Node(
        package='app_bridge',
        executable='app_bridge_node',
        name='app_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': 5000,
            'pose_publish_hz': 2.0,
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        camera_node,
        robot_driver_node,
        localizer_node,
        nav2_launch,
        app_bridge_node
    ])
