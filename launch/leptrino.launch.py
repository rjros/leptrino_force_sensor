"""
Copyright 2024, Ricardo Rosales Martinez

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

ROS2 Launch file for leprtrino 6 axis force/torque sensor
"""

# Author: [Ricardo Rosales Martinez]

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    com_port = DeclareLaunchArgument(name="com_port", default_value="/dev/ttyACM0")
    rate = DeclareLaunchArgument(name="rate", default_value="150")

    # Define parameters for the node
    parameters = {"com_port": LaunchConfiguration("com_port"), 
                  "rate": LaunchConfiguration("rate")}
    node_action = Node(
        package="leptrino_force_sensor",
        namespace="leptrino_force_sensor",
        executable="leptrino_force_sensor",
        output="screen",
        parameters=[parameters]  # Pass parameters to the node
    )

    return LaunchDescription([
        com_port,
        rate,
        node_action
    ])

