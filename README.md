# FindbotSimulator

Entorno de simulación en Docker con **ROS 2 Jazzy** + **Gazebo Harmonic** (con aceleración GPU NVIDIA).

## Requisitos
- Docker + Docker Compose
- NVIDIA Container Toolkit (para usar la GPU)

## Uso

```bash
# 1. Permitir que el contenedor abra ventanas en tu pantalla (una vez por sesión)
xhost +local:docker

# 2. Construir la imagen (solo la primera vez o si cambia el Dockerfile)
docker compose build

# 3. Levantar el contenedor y entrar
docker compose up -d
docker exec -it findbot_sim bash

# Dentro del contenedor:
gz sim shapes.sdf                 # Probar Gazebo
ros2 run demo_nodes_cpp talker    # Probar ROS 2
colcon build --symlink-install   # Compilar (ya arranca en usar_ws)

# 4. Detener
docker compose down
```

Todo el repositorio se monta en `/home/findbot/FindbotSimulator`; el workspace ROS 2 es `usar_ws/`.

> El archivo `.env` define `UID`/`GID` para que coincidan con tu usuario. Créalo con:
> `printf "UID=$(id -u)\nGID=$(id -g)\n" > .env`
