#!/usr/bin/env python3
import sys

import geometry_msgs.msg  # Импортируем сообщения для задания геометрических данных (скорости)
import rclpy  # Импортируем библиотеку для работы с ROS 2
import std_msgs.msg  # Импортируем стандартные сообщения ROS

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy  # Импортируем настройки QoS (качество сервиса) для публикации сообщений

# Проверяем операционную систему, чтобы выбрать соответствующий модуль для обработки ввода с клавиатуры
if sys.platform == 'win32':
    import msvcrt  # Модуль для работы с консолью на Windows
else:
    import termios  # Модуль для работы с терминалом на Unix-подобных системах
    import tty  # Модуль для управления настройками терминала на Unix

# Сообщение, которое выводится при запуске программы
msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

# Словарь для сопоставления клавиш с командами движения
moveBindings = {
    'w': (0, 0, 1, 0),  # Движение по оси Z вверх
    's': (0, 0, -1, 0), # Движение по оси Z вниз
    'a': (0, 0, 0, -1), # Вращение (Yaw) влево
    'd': (0, 0, 0, 1),  # Вращение (Yaw) вправо
    '\x1b[A' : (0, 1, 0, 0),  # Вперёд по оси Y (стрелка вверх)
    '\x1b[B' : (0, -1, 0, 0), # Назад по оси Y (стрелка вниз)
    '\x1b[C' : (-1, 0, 0, 0), # Вправо по оси X (стрелка вправо)
    '\x1b[D' : (1, 0, 0, 0),  # Влево по оси X (стрелка влево)
}

# Добавляем новые биндинги для изменения скорости
speedBindings = {
    'q': (1.1, 1.1),  # Увеличить линейную и угловую скорость на 10%
    'z': (0.9, 0.9),  # Уменьшить линейную и угловую скорость на 10%
    'e': (1.0, 1.1),  # Увеличить только угловую скорость на 10%
    'c': (1.0, 0.9),  # Уменьшить только угловую скорость на 10%
}

# Функция для получения нажатой клавиши с терминала
def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()  # Для Windows
    else:
        tty.setraw(sys.stdin.fileno())  # Переключаем терминал в "сырое" состояние (без буферизации ввода)
        key = sys.stdin.read(1)  # Читаем один символ
        if key == '\x1b':  # Если первый символ - это '\x1b', возможно, это стрелочная клавиша
            additional_chars = sys.stdin.read(2)  # Читаем еще два символа, чтобы получить полную клавишу
            key += additional_chars
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Возвращаем настройки терминала
    return key

# Функция для сохранения настроек терминала (только для Unix)
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

# Функция для восстановления настроек терминала (только для Unix)
def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# Функция для отображения текущей скорости и угла поворота
def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

# Основная функция
def main():
    settings = saveTerminalSettings()  # Сохраняем настройки терминала

    rclpy.init()  # Инициализируем ROS 2

    node = rclpy.create_node('teleop_twist_keyboard')  # Создаем узел ROS 2

    # Создаем профиль QoS для публикации сообщений с наилучшей возможной доставкой
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Политика надежности: лучший результат
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Политика долговечности: сообщения сохраняются локально до получения подписчиком
        history=QoSHistoryPolicy.KEEP_LAST,  # Политика истории: хранить только последние сообщения
        depth=10  # Глубина очереди сообщений
    )

    # Создаем издателя сообщений типа Twist на топик /offboard_velocity_cmd
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    arm_toggle = False  # Переменная для отслеживания состояния "арминга" дрона
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)  # Создаем издателя для арминга/разарминга дрона

    # Инициализируем переменные скорости и направления
    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    try:
        print(msg)  # Выводим инструкцию
        while True:
            key = getKey(settings)  # Получаем нажатую клавишу
            
            if key in moveBindings.keys():  # Проверяем, связана ли клавиша с движением
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                
            elif key in speedBindings.keys():
                # Изменение скорости при нажатии клавиш Q/Z/E/C
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(vels(speed, turn))  # Отображаем новые значения скорости и угловой скорости
                continue
            
            else:
                # Если клавиша не соответствует движению, сбрасываем значения
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if key == '\x03':  # Если нажата клавиша Ctrl+C, выходим из цикла
                    break

            if key == ' ':  # Проверяем, нажата ли пробел для переключения арминга
                arm_toggle = not arm_toggle  # Переключаем состояние арминга
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)  # Публикуем сообщение с новым состоянием арминга
                print(f"Arm toggle is now: {arm_toggle}")

            # Создаем сообщение Twist для задания линейных и угловых скоростей
            twist = geometry_msgs.msg.Twist()
            
            # Рассчитываем новые значения для скоростей на основе нажатых клавиш
            x_val = (x * speed) + x_val
            y_val = (y * speed) + y_val
            z_val = (z * speed) + z_val
            yaw_val = (th * turn) + yaw_val
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val

            pub.publish(twist)  # Публикуем сообщение Twist на соответствующий топик
            print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)  # Выводим текущее состояние

    except Exception as e:
        print(e)  # Выводим ошибку, если она произошла

    finally:
        # При завершении программы отправляем нулевые значения скоростей, чтобы остановить дрон
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)  # Публикуем сообщение остановки

        restoreTerminalSettings(settings)  # Восстанавливаем настройки терминала

# Запуск основной функции, если скрипт запускается напрямую
if __name__ == '__main__':
    main()
