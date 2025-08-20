FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y python3-pip ros-noetic-rosbridge-suite ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-std-msgs ros-noetic-diagnostic-msgs vim nano curl net-tools && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

RUN mkdir -p src

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace src"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

EXPOSE 9090
EXPOSE 11311

CMD ["bash"]
