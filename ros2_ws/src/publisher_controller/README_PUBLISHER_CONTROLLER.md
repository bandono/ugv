
# Controller ROS 2 Package

  

This package contains a ROS 2 node that reads input from a Daxa Asteria DX-AX1 controller connected via a dongle and publishes the state of the controller's axes and buttons to a ROS 2 topic.

  

## Prerequisites

  

- Ubuntu 22.04

- ROS 2 Humble

- Python 3

- Daxa Asteria DX-AX1 controller

  

## Installation

  

1. Ensure you have ROS 2 Humble installed. Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html) if you don't have it installed.

2. Clone this repository to your workspace:

  

	```

	mkdir -p ~/ros2_ws/src

	cd ~/ros2_ws/src

	git clone <repository_url>

	```

  

3. Build the Package

  

	```

	cd ~/ros2_ws

	colcon build

	```

  

## Usage

  

1. Connect your Daxa Asteria DX-AX1 controller to your computer using the dongle.

2. List available joystick devices:

  

	```

	ls /dev/input

	```

  

3. Ensure your controller appears as /dev/input/js0 or modify the script accordingly if it appears as a different device.

4. Source your ROS 2 setup script:

  

	```

	source /opt/ros/humble/setup.bash

	source ~/ros2_ws/install/setup.bash

	```

5. Run the controller node:

  

```

ros2 run <package_name> <node_executable>

```

Replace <package_name> with the name of your package and <node_executable> with the name of your node's executable.

Example:
```

ros2 run xbox_controller publisher

```

## Files
- `publisher.py`: The main script that reads input from the controller and publishes it to a ROS 2 topic.

- `__init__.py`: An empty file that marks this directory as a Python package.                         

## Node Details
### Subscibed Topics
None

### Publised Topics
- `controller` (`std_msgs/Float32MultiArray`): Publishes the state of the gas and brake axes.
                                                                                       
### Parameters
None

### Troubleshooting
-   If the controller is not detected, ensure it is properly connected and recognized by your system.
-   Verify the device file `/dev/input/js0` exists. If not, adjust the script to point to the correct device file.
