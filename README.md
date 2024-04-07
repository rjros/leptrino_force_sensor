# 6-Axis Leptrino Force Sensor for ROS 2
This repository contains ROS 2 packages for interfacing with the 6-axis force sensor from Leptrino. The sensor provides measurements for force and torque in six dimensions. The node publishes a wrench stamped message [force,torque] in N and N.m respectively. 

## Installation

To use this ROS 2 package, you need to follow these installation steps:

1. Clone this repository into your ROS 2 workspace:

    ```bash
    git clone https://github.com/your_username/6-axis-force-sensor.git
    ```

2. Build the ROS 2 packages:

    ```bash
    colcon build
    ```

3. Source your ROS 2 workspace:

    ```bash
    source install/setup.bash
    ```

## Usage

To use the 6-axis force sensor with ROS 2, follow these steps:

1. Connect the sensor to your system.

2. Launch the ROS 2 node:

    ```bash
    ros2 launch leptrino_force_sensor leptrino.launch.py 
    ```

3. Check the published topic as follows:
    ```bash
    ros2 topic echo /leptrino_force_sensor/leptrino_wrench_topic
    ```
## Additional Resources

- [Official Website of the 6-Axis Force Sensor](https://www.leptrino.co.jp/product/6axis-force-sensor)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)

## License

This project is licensed under the [Apache License 2.0](LICENSE).
