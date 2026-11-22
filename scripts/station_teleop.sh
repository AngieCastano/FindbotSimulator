#!/usr/bin/env bash
# Teleoperación con teclado DESDE LA ESTACIÓN REMOTA (mission.launch.py teleop:=true).
# Publica en /station/cmd_vel: el mando cruza el enlace simulado (comm_relay) y llega al robot
# por /cmd_vel_teleop, pasando por el filtro de colisión y el vigilante del firmware.
# Uso (otra terminal, con el mismo ROS_DOMAIN_ID): ./scripts/station_teleop.sh
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/station/cmd_vel \
  -p speed:=0.25 -p turn:=1.0
