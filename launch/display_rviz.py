from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_share = get_package_share_directory('aerial_drone_base')

    # Путь к файлу .xacro
    urdf_file = os.path.join(pkg_share, 'urdf', 'dron_base.xacro')
    
    # Путь к файлу конфигурации RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'base.rviz')

    return LaunchDescription([
        # Параметры для URDF
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument(name='robot_description', default_value=Command(['xacro ', urdf_file]), description='URDF XML'),

        # Загрузка описания модели в параметр сервера
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description')
            }]
        ),
        
        # Добавление joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Запуск RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
