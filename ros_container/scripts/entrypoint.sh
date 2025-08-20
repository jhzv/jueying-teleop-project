#!/bin/bash

# Configurar entorno ROS
source /opt/ros/noetic/setup.bash

# Si existe workspace compilado, cargarlo
if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

# Configurar ROS_MASTER_URI si no está definido
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_IP=${ROS_IP:-127.0.0.1}

echo "=== ROS 1 Noetic Container para Jueying Lite3 ==="
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"
echo "Workspace: /catkin_ws"
echo "========================================"

# Ejecutar comando pasado como argumento
exec "$@"
