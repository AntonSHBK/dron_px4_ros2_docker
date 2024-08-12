#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Название пакета
    pkg_name = 'delta_robot_ros2'
    
    pkg_share = get_package_share_directory(pkg_name)
    
    # Путь к файлу конфигурации RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", "visualize.rviz"]
    )    

    # Определение нод
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    visualizer_node = Node(
        package='delta_robot_ros2',
        namespace='delta_robot_ros2',
        executable='visualizer',
        name='visualizer'
    )
    
    processes_node = Node(
        package='delta_robot_ros2',
        namespace='delta_robot_ros2',
        executable='processes',
        name='processes',
        prefix='tmux new-session -d -s mysession "bash -c \'',  # Создание новой сессии tmux и выполнение команды
    )
    
    control_node = Node(
        package='delta_robot_ros2',
        namespace='delta_robot_ros2',
        executable='control',
        name='control',
        prefix='tmux new-window -t mysession:1 "bash -c \'',  # Создание нового окна tmux в существующей сессии
    )
    
    velocity_node = Node(
        package='delta_robot_ros2',
        namespace='delta_robot_ros2',
        executable='velocity_control',
        name='velocity'
    )
    
    # Список нод
    nodes = [
        rviz_node,
        visualizer_node,
        processes_node,
        control_node,
        velocity_node,
    ]
    
    return LaunchDescription(nodes)

if __name__ == "__main__":
    generate_launch_description()


# tmux attach-session -t mysession