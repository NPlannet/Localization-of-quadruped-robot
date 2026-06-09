FROM osrf/ros:jazzy-desktop-full
USER root
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID >/dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        usermod -l $USERNAME $EXISTING_USER; \
        groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
        usermod -d /home/$USERNAME -m $USERNAME; \
    else \
        groupadd --gid $USER_GID $USERNAME; \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        x11-apps \
        python3-pip \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-tf-transformations \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*\
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Python packages für YOLO mit korrekter CUDA Version
RUN pip install --break-system-packages \
        torch torchvision \
        --index-url https://download.pytorch.org/whl/cu124 \
    && pip install --break-system-packages \
        ultralytics \
        "numpy<2" \
    && pip uninstall opencv-python -y || true

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "cd /workspaces/robot_sim_sose 2>/dev/null || true" >> /home/$USERNAME/.bashrc \
    && echo "[ -f install/setup.bash ] && source install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "cbs() { colcon build --symlink-install && source install/setup.bash; }" >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

USER $USERNAME