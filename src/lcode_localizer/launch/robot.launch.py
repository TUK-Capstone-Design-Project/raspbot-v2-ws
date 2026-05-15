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
    camera_node2 = Node(
        package='camera_node2',
        executable='camera_node2',
        name='camera_node2',
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

    # 6. Nav2 Navigation only (amcl 제외)
    #    bringup_launch.py 는 내부적으로 localization_launch.py(amcl) 까지 띄워서
    #    lcode_localizer 와 map->odom TF 가 충돌함. navigation_launch.py 만 사용한다.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params,
            'autostart': 'True',
            'use_composition': 'False',
        }.items()
    )

    # 6-1. Map Server (bringup_launch.py 가 아니면 따로 띄워야 함)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml,
            'topic_name': 'map',
            'frame_id': 'map',
        }]
    )

    # 6-2. Map Server lifecycle 관리 (navigation_launch.py 쪽 lifecycle_manager 는
    #      map_server 를 관리하지 않음. 따라서 별도 lifecycle_manager 가 필요)
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }]
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
        camera_node2,
        robot_driver_node,
        localizer_node,
        map_server_node,
        map_lifecycle_manager,
        nav2_navigation_launch,
        app_bridge_node
    ])
