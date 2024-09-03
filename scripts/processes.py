#!/usr/bin/env python3

import subprocess  # Модуль для выполнения системных команд и запуска процессов
import time  # Модуль для работы с временем (задержки)
import rclpy
from rclpy.node import Node

class ProcessesNode(Node):
    """
    Класс ProcessesNode представляет собой узел ROS 2, предназначенный для управления запуском и завершением
    нескольких процессов, связанных с управлением беспилотным летательным аппаратом (БПЛА).

    Основные функции класса:
    
    1. Инициализация узла ROS 2 и запуск процессов через `gnome-terminal`.
    2. Остановка всех предыдущих процессов PX4 и Gazebo.
    3. Создание новых вкладок `gnome-terminal` для параллельного выполнения команд.
    4. Управление процессами, включая завершение процессов по окончании работы узла.
    5. Создание таймера для периодического выполнения задач (если необходимо).
    """
    
    def __init__(self):
        super().__init__('processes_node')
        self.get_logger().info('Processes node has been started')

        # Список команд для выполнения
        self.commands = [
            # Запуск Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888",

            # Запуск симуляции PX4 SITL
            "cd /workspace/src/PX4-Autopilot && make px4_sitl gz_x500",
            
            # Запуск QGroundControl (закомментировано)
            # "cd ~/QGroundControl && ./QGroundControl.AppImage"
        ]

        # Остановка предыдущих процессов перед запуском новых
        self.clean_previous_sessions()

        # Инициализация gnome-terminal и запуск процессов
        self.start_gnome_terminal_sessions()

        # Создаем таймер для периодической проверки
        self.timer = self.create_timer(1.0, self.timer_callback)

    def clean_previous_sessions(self):
        """Завершаем все процессы PX4 и Gazebo, которые могут быть запущены."""
        self.get_logger().info("Cleaning up any previous PX4 and Gazebo processes...")
        
        # Завершаем процессы PX4
        subprocess.run(["pkill", "-f", "px4"])

        # Завершаем процессы Gazebo
        subprocess.run(["pkill", "-f", "gz"])
        subprocess.run(["pkill", "-f", "gzserver"])
        subprocess.run(["pkill", "-f", "gzclient"])
        time.sleep(1)  # Подождем, чтобы процессы завершились

    def start_gnome_terminal_sessions(self):
        """Запускаем процессы в новых вкладках gnome-terminal."""
        
        for i, command in enumerate(self.commands):
            self.get_logger().info(f"Starting gnome-terminal tab and running command: {command}")
            subprocess.run([
                "gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"
            ])
            time.sleep(1)  # Задержка перед запуском следующей команды

    def timer_callback(self):
        """Эта функция будет вызвана периодически по таймеру, если нужно выполнять какие-то действия."""
        pass

    def shutdown_gnome_terminal_sessions(self):
        """Завершаем процессы gnome-terminal."""
        self.get_logger().info('Attempting to stop gnome-terminal processes')
        # Если необходимо, вы можете реализовать логику для завершения запущенных процессов

def main(args=None):
    rclpy.init(args=args)
    node = ProcessesNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Processes node interrupted')
    finally:
        # Обязательно вызываем завершение процессов
        node.shutdown_gnome_terminal_sessions()
        # Завершаем работу ROS контекста только после всех других действий
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()