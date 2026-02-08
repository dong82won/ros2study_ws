#! /usr/bin/env bash

# We set up the environment for ROS2
cd ~/ros2_ws/src/t3/turtlebot3_simulations/turtlebot3_gazebo
export GAZEBO_MODEL_PATH=./models:${GAZEBO_MODEL_PATH}
export TURTLEBOT3_MODEL=waffle

ros2 launch turtlebot3_gazebo empty_world.launch.py
