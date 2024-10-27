#!/usr/bin/env python3

from pathlib import Path
from datetime import datetime

import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude


class DroneDataLogger(Node):
    def __init__(self):
        super().__init__('drone_data_logger')
        
        # Создаем датафрейм для записи данных
        self.data = pd.DataFrame(columns=['position_x', 'position_y', 'position_z', 
                                          'velocity_x', 'velocity_y', 'velocity_z', 
                                          'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z', 'nav_state'])

        # Настраиваем QoS с параметром Best Effort
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Подписываемся на топики
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )
        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile
        )
        
        self.nav_state = None
        self.position_data = None
        self.attitude_data = None

        # Таймер для обновления данных
        self.timer = self.create_timer(1.0, self.log_data)

    def position_callback(self, msg):
        self.position_data = {
            'position_x': msg.x,
            'position_y': msg.y,
            'position_z': msg.z,
            'velocity_x': msg.vx,
            'velocity_y': msg.vy,
            'velocity_z': msg.vz
        }

    def attitude_callback(self, msg):
        # Проверяем, содержит ли сообщение атрибут `q` для ориентации
        if hasattr(msg, 'q'):
            self.attitude_data = {
                'orientation_w': msg.q[0],
                'orientation_x': msg.q[1],
                'orientation_y': msg.q[2],
                'orientation_z': msg.q[3]
            }
        else:
            self.get_logger().warn("Сообщение VehicleAttitude не содержит ориентационных данных.")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state

    def log_data(self):
        if self.position_data and self.attitude_data and self.nav_state is not None:
            # Создаем новую строку с данными
            row = pd.DataFrame([{
                'timestamp': datetime.now(),
                **self.position_data,
                **self.attitude_data,
                'nav_state': self.nav_state
            }])
            self.data = pd.concat([self.data, row], ignore_index=True)

    def save_data(self):
        # Округляем значения до двух знаков после запятой
        rounded_data = self.data.round(2)

        # Генерируем имя файла с минутой запуска
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M')
        Path('/workspace/src/dron_px4_ros2_docker/data/').mkdir(parents=True, exist_ok=True)
        file_path = f'/workspace/src/dron_px4_ros2_docker/data/drone_data_log_{timestamp}.csv'
        
        # Сохраняем датафрейм в файл
        rounded_data.to_csv(file_path, index=False)
        self.get_logger().info(f"Данные сохранены в {file_path}")

    def destroy_node(self):
        self.save_data()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DroneDataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Завершение работы по запросу пользователя (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()