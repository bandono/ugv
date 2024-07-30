# Motor Drive Using Twist Listener

## Environment

Tested using [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) running inside Docker on Ubuntu from [Jetson Nano Developer Kit SD Card Image](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write).

## Interfaces

L298N motor driver and [PCA9685](https://www.adafruit.com/product/815) PWM I2C interface connected to Jetson Nano. 

| L298N | PCA9685|
|:---|:---|
| ENA | jumper (active) |
| IN1| PWM(14)|
| IN2| PWM(15)|
| ENB | jumper (active) |
| IN3| PWM(12)|
| IN4| PWM(13)|

## Installation

Package creation

```
cd ~/ros2_ws/src/

ros2 pkg create  --build-type ament_python --license Apache-2.0 twist_listener
```

Modify and add files as committed to this `src` repo.

1. listener
   
   Simple twist message listener

2. mover
   
   Simple two motor drivers controlled using twist messages.

Building

```
    colcon build --packages-select twist_listener
```

## Running

```
cd ~/ros2_ws/

source install/setup.bash
```

run listener

```
ros2 run twist_listener listener
```

run mover

```
ros2 run twist_listener mover
```


