# Video Hub

## Pre-requisites
'''bash
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport
sudo apt install libopencv-dev
'''

## RUN
'''bash
export FASTRTPS_DEFAULT_PROFILES_FILE="config/shm_fastdds.xml"
ros2 run camera_publisher camera_publisher
'''
