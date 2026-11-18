#!/usr/bin/env bash
# Teleoperación con teclado a través del filtro de colisión (collision_monitor_teleop).
# Publica en /cmd_vel_teleop: el filtro frena el robot antes de chocar con obstáculos
# vistos por el LiDAR o la cámara de profundidad (incluidas las extremidades de las víctimas).
# Uso (con nav.launch.py corriendo): ./scripts/teleop.sh
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
