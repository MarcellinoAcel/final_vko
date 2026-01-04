#!/bin/bash

# Launch camera node
gnome-terminal --tab --title="RealSense Camera" -- bash -c "source install/setup.bash;
                                 ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p enable_gyro:=true \
  -p enable_accel:=true \
  -p enable_sync:=true \
  -p align_depth.enable:=true \
  -p unite_imu_method:=2;
                                echo Press any key to close;
                                read -n 1"


# Launch Orb SLAM3
gnome-terminal --tab --title="Orb SLAM3" -- bash -c "source install/setup.bash;
                                 ros2 run orb_slam3_ros2 orb_slam3_node /home/marcel/final_vko/src/orb_slam3_ros2/vocabulary/ORBvoc.txt /home/marcel/final_vko/src/orb_slam3_ros2/config/rgb-d/RealSense_D435i.yaml;
                                echo Press any key to close;
                                read -n 1"