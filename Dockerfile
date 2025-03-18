FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y git && apt install -y python3-pip && apt install -y python3-tk && apt install -y ros-noetic-urdf-geometry-parser && apt install -y ros-noetic-moveit

WORKDIR /ros_ws

RUN git clone --single-branch --depth 1 --branch main https://github.com/Jy-Lau/ROS-Hand-Gesture-Controlled-Shadow-Dexterous-Hand.git

RUN mv ROS-Hand-Gesture-Controlled-Shadow-Dexterous-Hand src

WORKDIR /ros_ws/src

RUN git submodule init && git submodule update

RUN pip3 install --no-cache-dir cvzone mediapipe

WORKDIR /ros_ws

RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
