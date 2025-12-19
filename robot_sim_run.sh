#!/bin/bash

# Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "source install/setup.bash;
                                 ros2 launch robot_gazebo gazebo.launch.py;
                                echo Press any key to close;
                                read -n 1"