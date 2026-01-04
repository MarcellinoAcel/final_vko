# ORB-SLAM3 Tutorial (ROS 2 / Colcon Workflow)

This tutorial explains how to set up **ORB-SLAM3** and build a ROS 2 project using `colcon`.

---

## System Requirements

- Ubuntu 22.04  
- ROS 2 (Humble recommended)  
- git, cmake, colcon  
- OpenCV (>= 4.x)  
- Eigen3  
- Python 3  

> ORB-SLAM3 is sensitive to dependency versions. Make sure Pangolin and OpenCV are installed correctly.

---

## Step 1: Install ORB-SLAM3

install the ORB-SLAM3 using this link https://github.com/UZ-SLAMLab/ORB_SLAM3.git

## Step 2: Install Pangolin

install the Pangolin using this

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

