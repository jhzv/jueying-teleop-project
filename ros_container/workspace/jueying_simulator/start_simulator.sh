#!/bin/bash
source /opt/ros/noetic/setup.bash
roscore &
sleep 3
python3 /catkin_ws/src/jueying_simulator/fake_message_transformer.py &
sleep 2
roslaunch rosbridge_server rosbridge_websocket.launch
