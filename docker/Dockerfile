# ROS 2 Jazzy (LTS) + Gazebo Harmonic
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=findbot
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        python3-rosdep \
        git nano sudo bash-completion mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Usuario no-root con el mismo UID/GID que el host (archivos del volumen con permisos correctos).
# La imagen base trae un usuario "ubuntu" con UID 1000; se elimina para evitar conflictos.
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/FindbotSimulator

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/FindbotSimulator/usar_ws/install/setup.bash ] && source ~/FindbotSimulator/usar_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
