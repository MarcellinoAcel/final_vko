#!/bin/bash

# Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "source install/setup.bash;
                                 ros2 launch robot_properties gazebo.launch.py gazebo_rviz:=false;
                                echo Press any key to close;
                                read -n 1"

# Launch Maze Solver Node
gnome-terminal --tab --title="Maze Solver" -- bash -c "source install/setup.bash;
                                 ros2 run maze_rl_pkg train;
                                echo Press any key to close;
                                read -n 1"