###################
# ROS Layer
###################
FROM ros:humble-ros-core-jammy AS ros     

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Setup ROS2 Foxy
RUN apt update -y && \
        apt install ros-dev-tools -y 
RUN rosdep init && rosdep update

# Extra dependencies
RUN apt update && apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav python3-pip -y
RUN apt update && apt install -y ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-robot-localization
RUN pip3 install ros2-numpy open3d pycolmap ahrs filterpy

###################
# Arduino Layer
###################
FROM ros AS arduino
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/root/bin:$PATH"

###################
# Dev Layer
###################
FROM arduino AS dev

ARG USERNAME=dev
ENV USERNAME=$USERNAME
RUN apt update && apt install -y bash bash-completion
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin

# Python Dev
RUN pip install --upgrade pip && pip install \
    ruff \
    black \
    mypy \
    flake8 \
    isort

# Set up a non-root user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
chown -R $USERNAME:$USERNAME /home/$USERNAME
RUN chsh -s /bin/bash $USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME/workspace

RUN echo "source /etc/bash_completion" >> /home/$USERNAME/.bashrc \
        &&  echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
        &&  echo "export PATH=$PATH:/usr/local/bin"

RUN chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc
