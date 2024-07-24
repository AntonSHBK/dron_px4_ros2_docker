import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    
    pkg_share = get_package_share_directory('aerial_drone_base')

    # Путь к файлу .xacro
    urdf_file = os.path.join(pkg_share, 'urdf', 'dron_base.xacro')
    
    # Путь к файлу конфигурации RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'base.rviz')
    
    # xacro_file = PathJoinSubstitution(
    #     [FindPackageShare("your_package_name"), "urdf", "your_robot.xacro"]
    # )
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("your_package_name"), "rviz", "your_robot.rviz"]
    # )

    return LaunchDescription([
        # Параметры для URDF
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument(name='robot_description', default_value=Command(['xacro ', urdf_file]), description='URDF XML'),        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ),
        ),
        
        # Launch robot_state_publisher
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
        # Launch RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file]
        ),
        # Launch spawn_entity to spawn robot in Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "aerial_drone_base",
                "-topic", "robot_description",
                "-x", "0.0",  # начальная позиция по X
                "-y", "0.0",  # начальная позиция по Y
                "-z", "1.0",  # начальная позиция по Z
                "-R", "0.0",  # начальный угол Roll
                "-P", "0.0",  # начальный угол Pitch
                "-Y", "0.0"   # начальный угол Yaw
            ],
            output="screen"
        )
    ])

if __name__ == "__main__":
    generate_launch_description()
