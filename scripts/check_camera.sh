#!/bin/bash
# Скрипт для проверки и настройки Raspberry Pi камеры

echo "Проверка настроек камеры Raspberry Pi"
echo "-------------------------------------"

# Проверяем, включена ли камера
if ! vcgencmd get_camera | grep -q "detected=1"; then
  echo "ОШИБКА: Камера не обнаружена!"
  echo "Проверьте подключение шлейфа камеры и убедитесь, что камера включена в raspi-config"
  echo "Выполните: sudo raspi-config > Interface Options > Camera > Enable"
  exit 1
else
  echo "Камера обнаружена: OK"
fi

# Проверяем, загружен ли модуль камеры
if ! lsmod | grep -q "^bcm2835_v4l2"; then
  echo "Загружаем модуль камеры V4L2..."
  sudo modprobe bcm2835-v4l2
  if [ $? -ne 0 ]; then
    echo "ОШИБКА: Не удалось загрузить модуль камеры!"
    exit 1
  fi
  echo "Модуль камеры загружен: OK"
else
  echo "Модуль камеры уже загружен: OK"
fi

# Проверяем, доступно ли устройство камеры
if [ ! -e /dev/video0 ]; then
  echo "ОШИБКА: Устройство камеры /dev/video0 не найдено!"
  exit 1
else
  echo "Устройство камеры /dev/video0 доступно: OK"
fi

# Проверяем права доступа к устройству
if [ ! -r /dev/video0 ] || [ ! -w /dev/video0 ]; then
  echo "Исправляем права доступа к устройству камеры..."
  sudo chmod a+rw /dev/video0
  echo "Права доступа исправлены: OK"
else
  echo "Права доступа к устройству камеры: OK"
fi

echo ""
echo "Проверка завершена. Камера должна работать корректно."
echo "Если проблемы остаются, попробуйте перезагрузить Raspberry Pi: sudo reboot"
echo ""
echo "Для тестирования камеры можно использовать команду:"
echo "rosrun image_view image_view image:=/raspicam_node/image"
echo ""
echo "Или для проверки через веб-браузер:"
echo "http://localhost:8080/stream?topic=/raspicam_node/image&type=mjpeg"
echo ""
echo "Для просмотра списка доступных топиков:"
echo "rostopic list | grep raspicam"

exit 0 