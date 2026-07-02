FROM ros:jazzy-ros-base

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG INSTALL_YOLO=false
ARG INSTALL_XGO_SDK=true

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV YOLO_DEVICE=cpu
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

USER root

RUN if id -u ${USERNAME} >/dev/null 2>&1; then \
        usermod -u ${USER_UID} ${USERNAME}; \
        groupmod -g ${USER_GID} ${USERNAME} 2>/dev/null || true; \
    elif id -u ${USER_UID} >/dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd ${USER_UID} | cut -d: -f1); \
        EXISTING_GROUP=$(getent group ${USER_GID} | cut -d: -f1); \
        usermod -l ${USERNAME} ${EXISTING_USER}; \
        if [ -n "${EXISTING_GROUP}" ]; then groupmod -n ${USERNAME} ${EXISTING_GROUP}; fi; \
        usermod -d /home/${USERNAME} -m ${USERNAME}; \
    else \
        if ! getent group ${USER_GID} >/dev/null 2>&1; then groupadd --gid ${USER_GID} ${USERNAME}; fi; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        curl \
        git \
        iproute2 \
        locales \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pil \
        python3-pip \
        sudo \
        vim-tiny \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-simple-commander \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-vision-msgs \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/* \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN if [ "${INSTALL_YOLO}" = "true" ]; then \
        pip install --break-system-packages torch torchvision ultralytics "numpy<2" \
        && pip uninstall opencv-python -y || true; \
    fi

RUN apt-get update \
    && for pkg in \
        ros-${ROS_DISTRO}-camera-ros \
        ros-${ROS_DISTRO}-ldlidar-stl-ros2; do \
        if apt-cache show "${pkg}" >/dev/null 2>&1; then \
            apt-get install -y --no-install-recommends "${pkg}"; \
        else \
            echo "WARNING: optional package ${pkg} is not available from apt for ${ROS_DISTRO}."; \
        fi; \
    done \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${INSTALL_XGO_SDK}" = "true" ]; then \
        pip install --break-system-packages \
            xgolib \
            pyserial \
            pyserial-asyncio \
            smbus2 \
            gpiozero \
            lgpio \
            rpi-lgpio; \
    fi

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "cd /workspaces/robot_sim_sose 2>/dev/null || true" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f install/setup.bash ] && source install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "alias robot-build='bash /workspaces/robot_sim_sose/scripts/robot_build_workspace.sh'" >> /home/${USERNAME}/.bashrc \
    && echo "alias robot-stack='bash /workspaces/robot_sim_sose/scripts/robot_stack.sh'" >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspaces/robot_sim_sose

CMD ["sleep", "infinity"]
