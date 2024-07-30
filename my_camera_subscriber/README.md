# Camera Subscriber to Receive Message From Camera Publisher (PC Side)

## Environtment
Tested using [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) running inside Docker on Ubuntu from [Jetson Nano Developer Kit SD Card Image](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write).
Camera used Kurokesu C1 X18
Using OpenCV to capture video feed and convert to msg compatible to ROS2 

## Installation 

Dependencies Installation 
'''
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-camera-info-manager
'''

Package creation
'''
ros2 pkg create --build-type ament_python my_camera_subscriber
'''

Change and Modify all code created as committed to this repo (specially package.xml and setup.py)

## Building 

Build using colcon 
'''
    colcon build
'''

## Running 
'''
    cd `/my_camera_subscriber 
    source install/setup.bash
    ros2 run my_camera_subscriber camera_subscriber
'''