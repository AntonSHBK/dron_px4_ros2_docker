#!/usr/bin/env python3

# Импортируем модули subprocess и time
import subprocess  # Модуль для выполнения системных команд и запуска процессов
import time  # Модуль для работы с временем (задержки)

# Список команд для выполнения
commands = [
    # Запуск Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Запуск симуляции PX4 SITL
    "cd /workspace/src/PX4-Autopilot && make px4_sitl gz_x500",

    # Запуск QGroundControl (закомментировано)
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Инициализируем tmux сессию
subprocess.run(["tmux", "new-session", "-d", "-s", "mysession"])

# Проход по каждому элементу в списке команд
for i, command in enumerate(commands):
    if i == 0:
        # Первая команда запускается в первом окне сессии tmux
        subprocess.run(["tmux", "send-keys", "-t", "mysession:0", command + "; exec bash", "C-m"])
    else:
        # Последующие команды запускаются в новых окнах tmux
        subprocess.run(["tmux", "new-window", "-t", f"mysession:{i}", "-n", f"window{i}", "bash", "-c", command + "; exec bash"])
    
    # Задержка перед запуском следующей команды (1 секунда)
    time.sleep(1)

# Команда для подключения к сессии tmux (по желанию)
# subprocess.run(["tmux", "attach-session", "-t", "mysession"])
