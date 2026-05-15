FROM osrf/ros:jazzy-desktop-full

# Use root to setup the user
USER root
# Generic user inside the container. This does not need to match the host username.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Rename an existing UID 1000 user, or create one, so mounted files are writable
# on the common Linux/WSL setup where the host user also has UID 1000.
RUN if id -u $USER_UID >/dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        usermod -l $USERNAME $EXISTING_USER; \
        groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
        usermod -d /home/$USERNAME -m $USERNAME; \
    else \
        groupadd --gid $USER_GID $USERNAME; \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc


# Force bash as the default shell for the user
#RUN chsh -s /bin/bash $USERNAME
#SHELL ["/bin/bash", "-c"]

USER $USERNAME
