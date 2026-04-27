# Carla Vehicle

A CARLA-ROS vehicle simulation framework designed for the generation of custom synthetic data and support for automated vehicles research.

## Folder Structure

* **`carla_vehicle/`**: Contains the implementation of the ROS-CARLA bridge using the Python CARLA API.
* **`config/`**: Contains configuration files for the vehicle's sensor setup and control.
* **`include/`**: Header files that implement several features for localization and control.
* **`launch/`**: Launch files in standard ROS format.
* **`rviz/`**: Visualization configuration files for RViz.
* **`scripts/`**: Main scripts to launch the ROS-CARLA Bridge.
* **`src/`**: Source code of the features for localization and control.

## Requirements

* **OS:** Ubuntu 22.04 LTS
* **ROS:** ROS2 Humble
* **Dependencies:** Ensure all individual package dependencies are met (*Check each dependency*).

## Source Features

* **`geo_converter_node`**: A node that takes NavSat fix messages and converts them to local coordinates in an odometry frame given an initial GNSS position.
* **`image_transport_node`**: A node that takes images generated from the CARLA server and adds a compression channel for capture.
* **`front_axis_transform`**: A node that transforms local positions to the control axis frame.
* **`ackermann_wheel_odometry`**: A node that generates wheel odometry based on Ackermann commands.

## Configuration

* **`global_params.yaml`**: Contains main global configurations for the vehicle to spawn properly, such as the transformation tree or the main topics for sensors and vehicle information.
* **`sensor_params.yaml`**: Contains the instances of each sensor that will be spawned and attached to the vehicle once the simulation is launched. It allows you to attach any of the following sensors: GPS, IMU, RGB_CAMERA, SEMANTIC_CAMERA, LIDAR, SEMANTIC_LIDAR, RADAR, and OBU. Any number of sensor items can be attached if instantiated inside the proper dictionaries. Note that you can set each sensor's parameters independently.
* **`control/`**: Contains configurations for the manual and Ackermann controllers. The Ackermann parameters are pre-adjusted for the default ego model (Toyota Prius).
* **Localization**: Configuration for the Unscented Kalman Filter (UKF) to provide filtered localization.

## Main Bridge

The bridge is implemented to align with standard ROS2 interfaces and transformation tree requirements. For the proper simulation of the vehicle, a robot description must first be launched, as the first thing the bridge does is listen to static transforms to properly build the complete `tf` tree. 

Key files include:

* **`vehicle_world.py`**: Implements and abstracts the CARLA world for management. It deploys custom services and attaches sensors to the vehicle based on the sensor configuration files. It also generates random hypotheses based on nearby players to trigger [Perception Events](https://github.com/lsiteam/perception_interfaces).
* **`vehicle_ros_bridge.py`**: Implements the vehicle bridge between CARLA and ROS. It reads the custom transformations provided through the previously launched robot description and generates a transformation tree that updates according to simulation data. Additionally, it parses the sensor configuration file to instantiate requested sensors. **Note:** A sensor only spawns if there is a static `tf` with the exact same name AND it is properly declared in this configuration file; otherwise, the parser will drop that sensor type. On each tick, this node processes control commands sent to the vehicle model and parses sensor data types to be properly published through ROS topics.

## Scripts & Main Launch

* **`camera_pose.py`**: A Pygame script to check where the user camera is located within the CARLA world. This is very useful for placing sensors accurately.
* **`carla_ackermann_node.py`**: Interacts with the vehicle command topic to control the vehicle model via Ackermann commands. It converts Ackermann commands into throttle and brake commands that are sent to the bridge. It implements two PIDs (for throttle and brake), and the PID values are fine-tuned to work with the Toyota Prius model.
* **`carla_keyboard_control_node.py`**: Implements a manual controller using keyboard actions. It publishes Ackermann commands and sends them to the `carla_ackermann_node` for conversion.
* **`vehicle_node.py`**: The main entry point of the ROS-CARLA Bridge. It parses parameters and launches the entire bridge sequence.

## Installation

```bash
cd ros2_ws
git clone https://github.com/dayaguec/carla_vehicle.git

# Return to workspace root to build
cd ../.. 
colcon build
```

## Usage

To launch the CARLA vehicle simulation, use the following ROS2 command:

```bash
ros2 launch carla_vehicle carla_vehicle.launch.py
```
