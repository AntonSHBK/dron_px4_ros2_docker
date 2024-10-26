cmake_minimum_required(VERSION 3.8)
project(aerial_drone_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# include directories
# include_directories(include)

# create a executable
# add_executable(dron_base src/dron_base.cpp)



# Install Python modules
install(
  PROGRAMS scripts/control.py
  PROGRAMS scripts/visualizer.py
  PROGRAMS scripts/processes.py
  PROGRAMS scripts/velocity_control.py
  PROGRAMS scripts/inform_node.py
  DESTINATION lib/${PROJECT_NAME}  # Указываем путь установки
)

# install(
#   TARGETS dron_base
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
