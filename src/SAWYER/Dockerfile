FROM ros:noetic
 
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
 
# -----------------------------
# 1. Install system + ROS deps
# -----------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    python3-rosdep \
    python3-wstool \
    python3-vcstools \
    # python3-argparse \
    python3-pip \
    ros-noetic-catkin \
    ros-noetic-xacro \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-cv-bridge \
    ros-noetic-control-msgs \
    # ros-noetic-joystick-drivers \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-trajectory-msgs \
    ros-noetic-rospy-message-converter \
    ros-noetic-moveit \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-msgs \
    ros-noetic-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*
 
RUN python3 -m pip install "numpy==1.17.4" "pyRobotiqGripper==1.0.1"

RUN python3 -m pip install pygame
 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 
# -----------------------------
# 2. Create catkin workspace
# -----------------------------
WORKDIR /root/catkin_ws
RUN mkdir -p src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
 
# -----------------------------
# 3. Sawyer + Intera SDK
# -----------------------------
RUN git clone https://github.com/RethinkRobotics/sawyer_robot.git /root/catkin_ws/src/sawyer_robot
 
# intera_common requires wstool rosinstall file
RUN printf '%s\n' \
    "- git:" \
    "    local-name: intera_common" \
    "    uri: https://github.com/RethinkRobotics/intera_common.git" \
    "    version: master" > /root/catkin_ws/src/sawyer.rosinstall
 
RUN cd /root/catkin_ws && \
    wstool init src /root/catkin_ws/src/sawyer.rosinstall && \
    wstool update -t src
 
RUN git clone https://github.com/RethinkRobotics/intera_sdk.git /root/catkin_ws/src/intera_sdk
 
# -----------------------------
# 4. Add Sawyer MoveIt
# -----------------------------
RUN cd /root/catkin_ws && \
    wstool merge -t src https://raw.githubusercontent.com/RethinkRobotics/sawyer_moveit/master/sawyer_moveit.rosinstall && \
    wstool update -t src
 
# -----------------------------
# 5. Add Robotiq
# -----------------------------
# RUN git clone https://github.com/ros-industrial/robotiq.git /root/catkin_ws/src/robotiq
 
# -----------------------------
# 6. Setup rosdep
# -----------------------------
RUN rosdep init || true
RUN rosdep update
RUN cd /root/catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y
 
# -----------------------------
# 7. Build catkin workspace
# -----------------------------
RUN bash -c "\
    source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -j$(nproc)"
 
 
# -----------------------------
# 8. Utilities
# -----------------------------
RUN apt-get update && apt-get install -y \
    net-tools nano vim iputils-ping \
    && rm -rf /var/lib/apt/lists/*
 
RUN cp /root/catkin_ws/src/intera_sdk/intera.sh /root/catkin_ws/
 
CMD ["bash"]
