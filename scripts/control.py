#!/usr/bin/env python3

import sys
import geometry_msgs.msg  # Сообщения для управления скоростью
import rclpy  # Основная библиотека для работы с ROS 2
import std_msgs.msg  # Стандартные сообщения ROS

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Определяем работу с клавиатурой для разных операционных систем
if sys.platform == 'win32':
    import msvcrt  # Для Windows
else:
    import termios  # Для Unix
    import tty  # Для Unix

# Инструкция для управления дроном
msg = """
Управление дроном через клавиатуру:

W: Вверх
S: Вниз
A: Рыскание влево
D: Рыскание вправо
Стрелка вверх: Вперёд
Стрелка вниз: Назад
Стрелка влево: Ролл влево
Стрелка вправо: Ролл вправо

Пробел: Армирование/Дезармирование дрона

Q: Увеличить линейную и угловую скорость
Z: Уменьшить линейную и угловую скорость
E: Увеличить угловую скорость
C: Уменьшить угловую скорость

Нажмите CTRL+C для выхода
"""

# Команды движения, сопоставленные с клавишами
moveBindings = {
    'w': (0, 0, 1, 0),
    's': (0, 0, -1, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
    '\x1b[A': (0, 1, 0, 0),  # Вперёд
    '\x1b[B': (0, -1, 0, 0),  # Назад
    '\x1b[C': (-1, 0, 0, 0),  # Вправо
    '\x1b[D': (1, 0, 0, 0),  # Влево
}

# Команды изменения скорости
speedBindings = {
    'q': (1.1, 1.1),  # Увеличить линейную и угловую скорость на 10%
    'z': (0.9, 0.9),  # Уменьшить линейную и угловую скорость на 10%
    'e': (1.0, 1.1),  # Увеличить угловую скорость на 10%
    'c': (1.0, 0.9),  # Уменьшить угловую скорость на 10%
}

# Получение нажатой клавиши
def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()  # Для Windows
    else:
        tty.setraw(sys.stdin.fileno())  # Ввод в "сырое" состояние
        key = sys.stdin.read(1)  # Читаем один символ
        if key == '\x1b':  # Проверяем нажатие стрелок
            additional_chars = sys.stdin.read(2)
            key += additional_chars
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Возвращаем настройки терминала
    return key

# Сохранение настроек терминала (Unix)
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

# Восстановление настроек терминала (Unix)
def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# Функция для отображения текущих скоростей и состояния (переписывание нескольких строк)
def print_status(speed, turn, arm_state, twist):
    # Очищаем 5 строк перед выводом новой информации
    print("\033[5F\033[J", end="")  # Очищаем последние 5 строк перед выводом обновлений
    status_message = (
        f"Текущие параметры:\n"
        f"Скорость: {speed:.2f}\tПоворот: {turn:.2f}\tАрмирование: {'Включено' if arm_state else 'Отключено'}\n"
        f"X: {twist.linear.x:.2f}   Y: {twist.linear.y:.2f}   Z: {twist.linear.z:.2f}\n"
        f"Yaw: {twist.angular.z:.2f}   Roll: {twist.angular.x:.2f}   Pitch: {twist.angular.y:.2f}"
    )
    print(status_message, end='\r')  # Обновляем вывод на той же строке

# Основная функция управления
def main():
    settings = saveTerminalSettings()
    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')  # Создаём ROS 2 узел

    # Профиль QoS для публикации сообщений
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    # Публикация команд Twist для управления дроном
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    arm_toggle = False  # Состояние арминга
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)  # Публикация команд арминга

    # Инициализация переменных скорости
    speed = 0.5
    turn = 0.2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0
    twist = geometry_msgs.msg.Twist()

    print(msg)  # Выводим инструкцию
    print("\033[F\033[K", end="")  # Убираем инструкцию и очищаем строку

    try:
        while True:
            key = getKey(settings)  # Получаем нажатие клавиш

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                # Изменение скорости
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                continue
            else:
                # Сброс значений движения
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if key == '\x03':  # Выход по Ctrl+C
                    break

            if key == ' ':  # Армирование/дезармирование пробелом
                arm_toggle = not arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)

            # Вычисление скоростей и углов
            x_val = (x * speed) + x_val
            y_val = (y * speed) + y_val
            z_val = (z * speed) + z_val
            yaw_val = (th * turn) + yaw_val

            # Обновление сообщения Twist
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val

            pub.publish(twist)  # Публикация команд

            # Обновление статуса на экране
            print_status(speed, turn, arm_toggle, twist)

    except Exception as e:
        print(f"Произошла ошибка: {e}")

    finally:
        # Остановка дрона при выходе
        twist = geometry_msgs.msg.Twist()
        pub.publish(twist)
        restoreTerminalSettings(settings)

# Запуск основной функции
if __name__ == '__main__':
    main()
