import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义启动参数
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='camera_display.rviz',
        description='RViz configuration file name'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )
    
    return LaunchDescription([
        rviz_config_arg,
        use_rviz_arg,
        
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera_node',
            parameters=[{
                'camera_serial': '00F26632041',
                'camera_ip': '',
                'frame_rate': 30.0,
                'exposure_time': 10000.0,
                'gain': 5.0,
                'pixel_format': 'BGR8'
            }],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('hik_camera'),
                'config',
                LaunchConfiguration('rviz_config')
            ])],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        )
    ])