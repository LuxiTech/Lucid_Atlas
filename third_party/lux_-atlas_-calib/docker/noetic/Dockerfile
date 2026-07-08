FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libpcl-dev \
    libusb-1.0-0-dev \
    python3-catkin-tools \
    python3-pip \
    ros-noetic-cv-bridge \
    ros-noetic-geometry-msgs \
    ros-noetic-image-geometry \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-nav-msgs \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-rosbag \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir rosbags

WORKDIR /catkin_ws
