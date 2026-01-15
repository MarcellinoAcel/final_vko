#!/bin/bash

# Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "source install/setup.bash;
                                 ros2 launch robot_properties gazebo.launch.py gazebo_rviz:=false;
                                echo Press any key to close;
                                read -n 1"

gnome-terminal --tab --title="Behavior Tree" -- bash -c "source install/setup.bash;
                                 ros2 run elton_control_pkg elton_control;
                                echo Press any key to close;
                                read -n 1"