#!/usr/bin/env python3

############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Импортируем необходимые библиотеки
import time  # Импортируем модуль для добавления пауз

import numpy as np  # Библиотека для работы с массивами и математическими функциями

import rclpy  # Основная библиотека для работы с ROS 2
from rclpy.node import Node  # Класс для создания узла ROS 2
from rclpy.clock import Clock  # Класс для работы с временем в ROS 2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy  # Настройки QoS

# Импортируем необходимые сообщения из пакетов PX4 и ROS
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleAttitude, VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool  # Импортируем Bool для обработки сообщений булевого типа

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')  # Инициализация узла ROS 2 с именем 'minimal_publisher'

        # Настройка QoS для подписок и публикаций
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Создаем подписку на топик /fmu/out/vehicle_status, который содержит информацию о статусе дрона.
        # Этот топик публикует сообщения типа VehicleStatus, такие как текущее состояние навигации и арминга.
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,  # Функция-обработчик для сообщений VehicleStatus
            qos_profile  # Профиль качества сервиса (QoS) для подписки
        )

        # Создаем подписку на топик /offboard_velocity_cmd, который используется для получения команд скорости в offboard-режиме.
        # Сообщения типа Twist содержат линейные и угловые скорости для управления движением дрона.
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,  # Функция-обработчик для сообщений Twist
            qos_profile
        )

        # Создаем подписку на топик /fmu/out/vehicle_attitude, который содержит информацию о текущей ориентации дрона (углы наклона, рыскания и т.д.).
        # Сообщения типа VehicleAttitude помогают отслеживать ориентацию дрона во время полета.
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,  # Функция-обработчик для сообщений VehicleAttitude
            qos_profile
        )

        # Создаем подписку на топик /arm_message, который принимает команды арминга/разарминга дрона.
        # Сообщения типа Bool используются для переключения состояния дрона между армированным (True) и разарминным (False).
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,  # Функция-обработчик для сообщений Bool (арминга/разарминга)
            qos_profile
        )

        # Создаем издателя для публикации команд управления режимом offboard на топик /fmu/in/offboard_control_mode.
        # Сообщения типа OffboardControlMode позволяют переключать дрон в offboard-режим управления, когда команда отправляется извне.
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile  # Профиль QoS для публикации сообщений
        )

        # Создаем издателя для публикации команд скорости (Twist) на топик /fmu/in/setpoint_velocity/cmd_vel_unstamped.
        # Этот топик используется для отправки команд скорости в offboard-режиме.
        self.publisher_velocity = self.create_publisher(
            Twist,
            '/fmu/in/setpoint_velocity/cmd_vel_unstamped',
            qos_profile
        )

        # Создаем издателя для публикации сообщений о траектории (TrajectorySetpoint) на топик /fmu/in/trajectory_setpoint.
        # Сообщения TrajectorySetpoint задают целевые скорости, позиции и ориентацию дрона для управления движением.
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        # Создаем издателя для отправки общих команд управления дроном (VehicleCommand) на топик /fmu/in/vehicle_command.
        # Эти команды включают арминга, взлета, посадки и другие действия дрона.
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            25  # Глубина очереди для QoS (определяет количество сообщений, которые могут находиться в очереди на отправку)
        )

        # Создаем таймер для периодической отправки команд на армирование
        arm_timer_period = 0.1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # Создаем таймер для цикла управления дроном
        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Инициализируем переменные состояния дрона и управления

        # Текущее состояние навигации дрона. Устанавливаем его в максимальное значение NAVIGATION_STATE_MAX для начала.
        # Это состояние будет обновляться при получении информации от VehicleStatus.
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Текущее состояние арминга дрона. Устанавливаем в ARMING_STATE_ARMED как стартовое значение.
        # Это значение будет обновляться по мере изменения статуса арминга дрона (например, при армировании/разарминге).
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED

        # Вектор для хранения текущей скорости дрона по осям X, Y и Z.
        # Используется для задания линейных скоростей при движении дрона.
        self.velocity = Vector3()

        # Переменная для хранения заданного значения угла рыскания (yaw).
        # Этот угол управляет вращением дрона вокруг вертикальной оси.
        self.yaw = 0.0

        # Переменная для хранения текущего реального угла рыскания дрона.
        # Это значение будет получено из сообщений VehicleAttitude.
        self.trueYaw = 0.0

        # Флаг, указывающий, находится ли дрон в offboard-режиме.
        # Offboard-режим позволяет внешним командам управлять движением дрона.
        self.offboardMode = False

        # Флаг для проверки выполнения предварительных условий для полета (flight checks).
        # Это может включать проверку сенсоров, состояния дрона и других систем перед взлетом.
        self.flightCheck = False

        # Счетчик команд, используемый для отслеживания количества отправленных команд или состояния выполнения.
        # Может использоваться для временного отслеживания состояний, таких как ожидание успешного арминга.
        self.myCnt = 0

        # Переменная, указывающая, включен ли арминг дрона (True) или выключен (False).
        # Это значение будет управляться через топик /arm_message.
        self.arm_message = False

        # Флаг для активации failsafe-режима, если возникнет ошибка или сбой.
        # Failsafe используется для предотвращения опасных ситуаций (например, когда теряется управление дроном).
        self.failsafe = False


        # Определяем состояния и соответствующие функции, которые вызываются при смене состояния
        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "TAKEOFF": self.state_takeoff,
            "LOITER": self.state_loiter,
            "OFFBOARD": self.state_offboard
        }
        self.current_state = "IDLE"  # Начальное состояние
        self.last_state = self.current_state  # Предыдущее состояние

    # Callback-функция для обработки сообщений об арминге дрона
    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Получено сообщение арминга: {self.arm_message}")

    # Callback-функция для управления состояниями дрона (автомат)
    def arm_timer_callback(self):
        # self.get_logger().info(f"Текущее состояние: {self.current_state}, состояние арминга: {self.arm_state}, навигационное состояние: {self.nav_state}")

        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message:
                    self.current_state = "ARMING"
                    self.get_logger().info("Переход в состояние АРМИНГ")

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Проверка полета не пройдена, переход в состояние IDLE")
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10:
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("Армирование успешно, переход в состояние ВЗЛЕТ")
                self.arm()  # Отправляем команду на армирование

            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Проверка полета не пройдена, переход в состояние IDLE")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.current_state = "LOITER"
                    self.get_logger().info("Взлет успешен, переход в состояние LOITER")
                self.arm()  # Отправляем команду на армирование
                self.take_off()  # Отправляем команду на взлет

            case "LOITER": 
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Проверка полета не пройдена, переход в состояние IDLE")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("Лоитер успешен, переход в состояние OFFBOARD")
                self.arm()

            case "OFFBOARD":
                if not self.flightCheck or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                    self.current_state = "IDLE"
                    self.get_logger().info("Offboard режим не выполнен, переход в состояние IDLE")
                self.state_offboard()

        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(f"Состояние изменено на: {self.current_state}")

        self.myCnt += 1

    # Функции для инициализации состояния
    def state_init(self):
        self.myCnt = 0
        self.get_logger().info("Инициализация состояния")

    # Функция для состояния ARMED
    def state_arming(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Отправка команды армирования")

    # Функция для состояния TAKEOFF
    def state_takeoff(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5)  # param7 - высота в метрах
        self.get_logger().info("Отправка команды взлёта")

    # Функция для состояния LOITER
    def state_loiter(self):
        self.myCnt = 0
        self.get_logger().info("Статус лоитера: ожидание перехода")

    # Функция для состояния OFFBOARD
    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True        
        self.get_logger().info("Включен Offboard режим")

    # Функция для отправки команды на армирование
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Команда армирования отправлена")

    # Функция для отправки команды на взлет на заданную высоту
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)  # param7 - высота в метрах
        self.get_logger().info("Команда взлёта отправлена")

    # Публикует команду на топик /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1  # Первичный параметр команды. Его значение зависит от типа команды (например, для команды армирования это может быть 1.0 для арма и 0.0 для дизарма)
        msg.param2 = param2  # Вторичный параметр команды. Используется в некоторых командах для дополнительной информации (например, для некоторых команд это может быть направление или время)
        msg.param7 = param7  # Значение высоты для команды на взлет (например, при взлёте указывает желаемую высоту в метрах)
        msg.command = command  # ID команды (например, команда армирования, взлёта, перехода в Offboard и т.д.)
        msg.target_system = 1  # Система, которая должна выполнить команду (обычно 1 для основного контроллера)
        msg.target_component = 1  # Компонент, который должен выполнить команду (обычно 1 для основного компонента)
        msg.source_system = 1  # Система, отправляющая команду (указываем 1, если команда отправляется с основного контроллера)
        msg.source_component = 1  # Компонент, отправляющий команду (обычно 1)
        msg.from_external = True  # Флаг, показывающий, что команда отправлена извне
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # Время отправки команды в микросекундах
        self.vehicle_command_publisher_.publish(msg)  # Публикация команды
        # self.get_logger().info(f"Published VehicleCommand: command={command}, param1={param1}, param2={param2}, param7={param7}")


    # Callback-функция для получения и установки значений статуса дрона
    def vehicle_status_callback(self, msg):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATUS изменён: {msg.nav_state}")
        
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM_STATUS изменён: {msg.arming_state}")
            
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"Статус FAILSAFE изменён: {msg.failsafe}")
        
        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"Статус проверки полета изменён: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    # Callback-функция для обработки команд Twist из Teleop и преобразования их в систему координат FLU
    def offboard_velocity_callback(self, msg):
        # Логирование полученного сообщения Twist
        self.get_logger().info(f"Получено сообщение Twist: линейные=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), угловые=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")
        
        # Преобразование NED (North-East-Down) -> FLU (Forward-Left-Up)
        # В системе NED оси координат:
        # X - вперед, Y - вправо, Z - вниз
        # В системе FLU:
        # X - вперед, Y - влево, Z - вверх

        # Скорости по осям X, Y, Z
        self.velocity.x = msg.linear.x  # Скорость вперед (FLU X)
        self.velocity.y = -msg.linear.y  # Скорость влево (FLU -Y)
        self.velocity.z = -msg.linear.z  # Скорость вверх (FLU -Z)

        # Угловые скорости по осям X, Y, Z
        # Эти углы определяют вращение дрона вокруг каждой из осей:
        # X (roll) - вращение вокруг продольной оси дрона
        # Y (pitch) - вращение вокруг поперечной оси дрона
        # Z (yaw) - вращение вокруг вертикальной оси дрона
        self.roll = msg.angular.x  # Roll (вращение вокруг оси X)
        self.pitch = msg.angular.y  # Pitch (вращение вокруг оси Y)
        self.yaw = msg.angular.z  # Yaw (вращение вокруг оси Z)

        # Логируем преобразованные значения для проверки
        self.get_logger().info(f"Преобразовано в координатную систему FLU: скорость=({self.velocity.x}, {self.velocity.y}, {self.velocity.z}), roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")


    # Callback-функция для получения текущих значений траектории и извлечения угла рыскания
    def attitude_callback(self, msg):
        orientation_q = msg.q

        # trueYaw - текущее значение угла рыскания дрона
        self.trueYaw = -(np.arctan2(2.0 * (orientation_q[3] * orientation_q[0] + orientation_q[1] * orientation_q[2]), 
                                   1.0 - 2.0 * (orientation_q[0] * orientation_q[0] + orientation_q[1] * orientation_q[1])))

        # self.get_logger().info(f"Received attitude: trueYaw={self.trueYaw}")
        
    # Callback-функция для публикации режимов управления Offboard и скорости в качестве точек траектории
    def cmdloop_callback(self):
        if self.offboardMode:
            # Публикуем режимы управления Offboard
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)
            self.get_logger().info("Опубликован OffboardControlMode")

            # Вычисляем скорость в системе координат мира
            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # Создаем и публикуем сообщение TrajectorySetpoint с NaN значениями для позиции и ускорения
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = velocity_world_x
            trajectory_msg.velocity[1] = velocity_world_y
            trajectory_msg.velocity[2] = self.velocity.z
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_msg)
            self.get_logger().info(f"Опубликован TrajectorySetpoint: скорость=({velocity_world_x}, {velocity_world_y}, {self.velocity.z}), yaw={self.yaw}")

# Главная функция для запуска узла
def main(args=None):
    rclpy.init(args=args)  # Инициализация ROS 2

    offboard_control = OffboardControl()  # Создание экземпляра класса OffboardControl

    rclpy.spin(offboard_control)  # Запуск цикла обработки событий ROS 2

    offboard_control.destroy_node()  # Завершение работы узла
    rclpy.shutdown()  # Остановка ROS 2

if __name__ == '__main__':
    main()  # Запуск главной функции
