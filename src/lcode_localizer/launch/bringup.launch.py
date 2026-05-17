from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node2 = Node(
        package='camera_node2',
        executable='camera_node2',
        name='camera_node2',
        output='screen',
    )

    driver_node = Node(
        package='driver_node',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=[{
            'speed_scale': 40.0,
            'min_pwm': 15,
            'pwm_deadzone': 5,
            'max_pwm': 255,
            'cmd_timeout': 2.0,
        }],
    )

    lcode_localizer_node = Node(
        package='lcode_localizer',
        executable='lcode_localizer_node',
        name='lcode_localizer_node',
        output='screen',
    )

    return LaunchDescription([
        camera_node2,
        driver_node,
        lcode_localizer_node,
    ])
