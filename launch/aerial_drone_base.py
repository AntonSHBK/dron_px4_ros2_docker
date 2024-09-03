# !/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Название пакета
    pkg_name = 'aerial_drone_base'
    
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
        package='aerial_drone_base',
        namespace='aerial_drone_base',
        executable='visualizer.py',
        name='visualizer'
    )
    
    processes_node = Node(
        package=pkg_name,
        namespace='aerial_drone_base',
        executable='processes.py',
        name='processes',
        output='screen',
    )
    
    control_node = Node(
        package='aerial_drone_base',
        namespace='aerial_drone_base',
        executable='control.py',
        name='control',
        prefix="gnome-terminal --",
    )
    
    velocity_node = Node(
        package='aerial_drone_base',
        namespace='aerial_drone_base',
        executable='velocity_control.py',
        name='velocity',
        prefix="gnome-terminal --",
    )
    
    # Список нод
    nodes = [
        # rviz_node,
        # visualizer_node,
        processes_node,
        control_node,
        velocity_node,
    ]
    
    return LaunchDescription(nodes)

if __name__ == "__main__":
    generate_launch_description()


# tmux attach-session -t mysession