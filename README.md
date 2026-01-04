# ORB-SLAM3 Tutorial (ROS 2 / Colcon Workflow)

This tutorial explains how to set up **ORB-SLAM3** and build a ROS 2 project using `colcon`.

---

## System Requirements

- Ubuntu 20.04 / 22.04  
- ROS 2 (Humble recommended)  
- git, cmake, colcon  
- OpenCV (>= 4.x)  
- Eigen3  
- Python 3  

> ORB-SLAM3 is sensitive to dependency versions. Make sure Pangolin and OpenCV are installed correctly.

---

## Step 1: Clone and Build ORB-SLAM3

Clone the official ORB-SLAM3 repository:

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
