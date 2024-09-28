#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus, VehicleAngularVelocity, SensorGps
from sensor_msgs.msg import BatteryState
import time


class DroneInfoNode(Node):

    def __init__(self):
        super().__init__('drone_info_node')
        
        # Профиль качества обслуживания (QoS)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Подписка на топики
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile
        )
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/fmu/out/battery_status',
            self.battery_callback,
            qos_profile
        )
        self.angular_velocity_sub = self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.angular_velocity_callback,
            qos_profile
        )
        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile
        )

        # Переменные для хранения данных
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.orientation_quat = {'q0': 0.0, 'q1': 0.0, 'q2': 0.0, 'q3': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.arming_state = False
        self.nav_state = 'Unknown'
        self.battery = {'voltage': 0.0, 'current': 0.0, 'percentage': 0.0}
        self.angular_velocity = {'roll_rate': 0.0, 'pitch_rate': 0.0, 'yaw_rate': 0.0}
        self.gps_data = {'satellites': 0, 'fix_type': 'No Fix'}
        self.failsafe = False
        self.flight_time = 0.0

        # Таймер для времени полета
        self.start_time = time.time()

        # Таймер для обновления информации каждую секунду
        self.timer = self.create_timer(1.0, self.timer_callback)

    def local_position_callback(self, msg):
        """Получение координат дрона и высоты"""
        self.position['x'] = msg.x
        self.position['y'] = msg.y
        self.position['z'] = msg.z  # Высота дрона

    def attitude_callback(self, msg):
        """Получение ориентации дрона в пространстве (кватернионы и углы Эйлера)"""
        # Кватернионы
        self.orientation_quat['q0'] = msg.q[0]
        self.orientation_quat['q1'] = msg.q[1]
        self.orientation_quat['q2'] = msg.q[2]
        self.orientation_quat['q3'] = msg.q[3]

        # Углы Эйлера
        q0, q1, q2, q3 = msg.q
        self.orientation['roll'] = np.arctan2(2.0 * (q3 * q0 + q1 * q2), 1.0 - 2.0 * (q0 * q0 + q1 * q1))
        self.orientation['pitch'] = np.arcsin(2.0 * (q3 * q1 - q2 * q0))
        self.orientation['yaw'] = np.arctan2(2.0 * (q3 * q2 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2))

    def status_callback(self, msg):
        """Получение состояния дрона (арминга, failsafe и навигации)"""
        self.arming_state = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        nav_state_dict = {
            0: 'MANUAL',
            1: 'ALTCTL',
            2: 'POSCTL',
            3: 'AUTO_MISSION',
            4: 'AUTO_LOITER',
            5: 'AUTO_RTL',
            6: 'OFFBOARD',
            7: 'STABILIZED'
        }
        self.nav_state = nav_state_dict.get(msg.nav_state, 'Unknown')
        self.failsafe = msg.failsafe

    def battery_callback(self, msg):
        """Получение данных о состоянии батареи"""
        self.battery['voltage'] = msg.voltage
        self.battery['current'] = msg.current
        self.battery['percentage'] = msg.percentage * 100  # В процентах

    def angular_velocity_callback(self, msg):
        """Получение угловых скоростей дрона"""
        self.angular_velocity['roll_rate'] = msg.xyz[0]  # Roll rate (вокруг оси X)
        self.angular_velocity['pitch_rate'] = msg.xyz[1]  # Pitch rate (вокруг оси Y)
        self.angular_velocity['yaw_rate'] = msg.xyz[2]  # Yaw rate (вокруг оси Z)

    def gps_callback(self, msg):
        """Получение данных о состоянии GPS"""
        self.gps_data['satellites'] = msg.satellites_used
        gps_fix_dict = {0: 'No Fix', 1: '3D Fix', 2: 'DGPS Fix'}
        self.gps_data['fix_type'] = gps_fix_dict.get(msg.fix_type, 'Unknown')

    def timer_callback(self):
        """Функция, которая выводит данные в терминал с обновлением"""
        # Очищаем текущий вывод
        print("\033[2J\033[H", end="")

        # Время полета
        self.flight_time = time.time() - self.start_time

        # Выводим данные о состоянии дрона
        print(f"=== Параметры дрона ===")
        print(f"Время полета: {self.flight_time:.1f} секунд")
        print(f"Координаты (м): X: {self.position['x']:.3f}, Y: {self.position['y']:.3f}, Z: {self.position['z']:.3f}")
        print(f"Ориентация (рад): Roll: {self.orientation['roll']:.3f}, Pitch: {self.orientation['pitch']:.3f}, Yaw: {self.orientation['yaw']:.3f}")
        print(f"Ориентация (кватернионы): q0: {self.orientation_quat['q0']:.3f}, q1: {self.orientation_quat['q1']:.3f}, q2: {self.orientation_quat['q2']:.3f}, q3: {self.orientation_quat['q3']:.3f}")
        print(f"Скорости угловые (рад/с): Roll rate: {self.angular_velocity['roll_rate']:.3f}, Pitch rate: {self.angular_velocity['pitch_rate']:.3f}, Yaw rate: {self.angular_velocity['yaw_rate']:.3f}")
        print(f"Заряд батареи: {self.battery['percentage']:.1f}% (напряжение: {self.battery['voltage']:.2f} В, ток: {self.battery['current']:.2f} А)")
        print(f"GPS: Спутники: {self.gps_data['satellites']}, Тип фиксации: {self.gps_data['fix_type']}")
        print(f"Состояние арминга: {'ARMED' if self.arming_state else 'DISARMED'}")
        print(f"Режим навигации: {self.nav_state}")
        print(f"Failsafe: {'Активирован' if self.failsafe else 'Не активирован'}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
