# Commands

```bush

colcon build --packages-select aerial_drone_base

colcon build --packages-select px4_msgs px4_ros_com

ros2 launch aerial_drone_base aerial_drone_base.py

ign gazebo

make px4_sitl_default gz_x500

ros2 launch px4_ros_com sensor_combined_listener.launch.py

/opt/qgroundcontrol/squashfs-root/AppRun

/opt/QGroundControl.AppImage

MicroXRCEAgent udp4 -p 8888

reboot now

ros2 topic echo /fmu/out/vehicle_status


ENV PX4_HOME_LAT=50
ENV PX4_HOME_LON=23
ENV PX4_HOME_ALT=100

docker inspect -f ros_2_humle-ros-humble-master-1

export PX4_SIM_HOSTNAME=172.26.176.1

Обязательно проверить версии PX4 и px4_msg

```