FROM ros:jazzy-ros-base

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG INSTALL_XGO_SDK=true
ARG REQUIRE_LIDAR_DRIVER=true
ARG LIBCAMERA_RPI_TAG=v0.7.1+rpt20260429

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV PYTHONUNBUFFERED=1

USER root

RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/00no-sandbox

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
        cmake \
        curl \
        git \
        iproute2 \
        libdrm-dev \
        libgnutls28-dev \
        libjsoncpp-dev \
        libssl-dev \
        libudev-dev \
        libyaml-dev \
        locales \
        meson \
        ninja-build \
        pkg-config \
        python3-colcon-common-extensions \
        python3-jinja2 \
        python3-numpy \
        python3-pil \
        python3-pip \
        python3-ply \
        python3-vcstool \
        python3-yaml \
        sudo \
        udev \
        usbutils \
        v4l-utils \
        vim-tiny \
        ros-${ROS_DISTRO}-camera-ros \
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

# Build a single Raspberry Pi-flavored libcamera stack in /opt/ros/jazzy so
# camera_ros, libcamera helpers, and ROS runtime all use the same binaries.
RUN git clone --depth 1 --branch "${LIBCAMERA_RPI_TAG}" \
        https://github.com/raspberrypi/libcamera.git /tmp/libcamera \
    && cd /tmp/libcamera \
    && meson setup build \
        --prefix=/opt/ros/${ROS_DISTRO} \
        --libdir=lib \
        --buildtype=release \
        -Dpipelines=rpi/vc4 \
        -Dipas=rpi/vc4 \
        -Dgstreamer=disabled \
        -Dtest=false \
        -Ddocumentation=disabled \
        -Dcam=disabled \
        -Dlc-compliance=disabled \
        -Dqcam=disabled \
        -Dtracing=disabled \
        -Dpycamera=disabled \
    && meson install -C build \
    && ldconfig \
    && rm -rf /tmp/libcamera

RUN apt-get update \
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
