#!/bin/bash

# Разрешение для локальных подключений Docker к X-серверу
xhost +local:docker || true

# Получаем абсолютный путь к рабочей директории
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
#echo "ROOT_DIR: $ROOT_DIR"  # Печать пути для отладки

# Загружаем образ, если он не существует
docker pull ilya9kkk/ros_itmo:base >&1

docker stop ros_pc
docker rm ros_pc

# Запуск контейнера с необходимыми переменными и настройками
docker run -ti --rm \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e XAUTHORITY \
    -v /dev:/dev \
    -v "$ROOT_DIR/workspace:/workspace" \
    --net=host \
    --privileged \
    --name ros_pc ilya9kkk/ros_itmo:base
