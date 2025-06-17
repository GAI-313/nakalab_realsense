ARG ROS
FROM gai313/ros2:${ROS}
ARG ROS


# install depends, build tools
RUN apt-get update && apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    git wget cmake build-essential \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at unzip &&\
    rm -rf /var/lib/apt/lists/* 

WORKDIR /ws/src
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.54.2.zip &&\
    wget https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/4.51.1.zip &&\
    unzip 4.51.1.zip && unzip v2.54.2.zip &&\
    rm 4.51.1.zip && rm v2.54.2.zip

RUN cd librealsense-2.54.2 &&\
    mkdir build && cd build && cmake ../ &&\
    sudo make uninstall && make clean && make && sudo make install
    

# install ros dependencies
WORKDIR /ws/src/nakalab_realsense
RUN . /opt/ros/${ROS}/setup.bash &&\
    apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
	ros-humble-image-transport \
	ros-humble-diagnostic-updater &&\
    rm -rf /var/lib/apt/lists/* 

COPY package.xml package.xml
COPY launch launch
COPY params params
COPY urdf urdf
COPY rviz rviz
COPY CMakeLists.txt CMakeLists.txt
WORKDIR /ws
RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install
