#!/bin/bash

# Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "source install/setup.bash;
                                 ros2 launch robot_properties gazebo.launch.py;
                                echo Press any key to close;
                                read -n 1"

# gnome-terminal --tab --title="Orb SLAM3" -- bash -c "source install/setup.bash;
#                                  ros2 run orb_slam3_ros2 orb_slam3_node /home/marcel/final_vko/src/orb_slam3_ros2/vocabulary/ORBvoc.txt /home/marcel/final_vko/src/orb_slam3_ros2/config/rgb-d/RealSense_D435i.yaml;
#                                 echo Press any key to close;
#                                 read -n 1"jw