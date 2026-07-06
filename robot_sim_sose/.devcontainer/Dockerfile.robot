FROM ros:jazzy-ros-base

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG INSTALL_XGO_SDK=true
ARG REQUIRE_LIDAR_DRIVER=true

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
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
        gnupg2 \
        iproute2 \
        locales \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pil \
        python3-pip \
        sudo \
        v4l-utils \
        vim-tiny \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-simple-commander \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-image-transport-plugins \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/* \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN curl -fsSL https://archive.raspberrypi.org/debian/raspberrypi.gpg \
        | gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.org/debian/ bookworm main" \
        > /etc/apt/sources.list.d/raspi.list \
    && printf "Package: *\nPin: origin archive.raspberrypi.org\nPin-Priority: 1001\n" \
        > /etc/apt/preferences.d/rpi-pin

RUN apt-get update \
    && for pkg in \
        libcamera-apps \
        libcamera-dev; do \
        if apt-cache show "${pkg}" >/dev/null 2>&1; then \
            apt-get install -y --no-install-recommends "${pkg}"; \
        else \
            echo "WARNING: optional package ${pkg} is not available from apt for this base image."; \
        fi; \
    done \
    && if apt-cache show "ros-${ROS_DISTRO}-camera-ros" >/dev/null 2>&1; then \
        apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-camera-ros"; \
    else \
        echo "WARNING: optional package ros-${ROS_DISTRO}-camera-ros is not available from apt for ${ROS_DISTRO}."; \
    fi \
    && if apt-cache show "ros-${ROS_DISTRO}-ldlidar-stl-ros2" >/dev/null 2>&1; then \
        apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-ldlidar-stl-ros2"; \
    else \
        echo "WARNING: optional package ros-${ROS_DISTRO}-ldlidar-stl-ros2 is not available from apt for ${ROS_DISTRO}."; \
    fi \
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
