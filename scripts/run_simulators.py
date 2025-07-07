#!/usr/bin/env python

import os
import sys
import argparse

def main():
    """
    Главная функция для запуска различных симуляторов
    """
    parser = argparse.ArgumentParser(description='Запуск симуляторов дрона')
    parser.add_argument('simulator', choices=['standard', 'tuning', 'animate'],
                      help='Тип симулятора: standard - обычный симулятор, tuning - настройка PID, animate - анимированный симулятор')
    parser.add_argument('--scenario', '-s', type=int, default=0,
                      help='Индекс сценария для обычного симулятора (0-4)')
    
    args = parser.parse_args()
    
    # Проверяем, что мы находимся в директории скриптов
    current_dir = os.path.basename(os.getcwd())
    if current_dir != 'scripts':
        script_dir = os.path.join(os.getcwd(), 'scripts')
        if os.path.exists(script_dir):
            os.chdir(script_dir)
            print(f"Переход в директорию: {script_dir}")
        else:
            print("Ошибка: не удалось найти директорию 'scripts'")
            return
    
    # Проверяем наличие необходимых файлов
    required_files = {
        'standard': 'drone_simulator_standalone.py',
        'tuning': 'pid_tuning_simulator.py',
        'animate': 'drone_simulator_standalone.py'
    }
    
    if not os.path.exists(required_files[args.simulator]):
        print(f"Ошибка: файл {required_files[args.simulator]} не найден")
        return
        
    if not os.path.exists('pid_standalone.py'):
        print("Ошибка: файл pid_standalone.py не найден")
        return
    
    # Запускаем выбранный симулятор
    if args.simulator == 'standard':
        print(f"Запуск стандартного симулятора со сценарием {args.scenario}...")
        os.system(f"python drone_simulator_standalone.py {args.scenario}")
    elif args.simulator == 'tuning':
        print("Запуск симулятора настройки PID...")
        os.system("python pid_tuning_simulator.py")
    elif args.simulator == 'animate':
        print(f"Запуск анимированного симулятора со сценарием {args.scenario}...")
        os.system(f"python drone_simulator_standalone.py {args.scenario} animate")
    
    print("Симуляция завершена.")

if __name__ == "__main__":
    main() 