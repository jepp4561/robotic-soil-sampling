from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory("soil_sampler_bringup"),
        "config",
        "soil_sampler.yaml",
    )

    soil_sampler_experiment_path = os.path.join(
        get_package_share_directory("soil_sampler_experiment"),
        "launch",
        "experiment.launch.py",
    )

    soil_sampler_experiment_launch = ExecuteProcess(
        cmd=["ros2", "launch", soil_sampler_experiment_path],
        output="screen",
    )

    pico_port = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6612483CB5D9E2B-if00"

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "micro_ros_agent",
                    "micro_ros_agent",
                    "serial",
                    "--dev",
                    pico_port,
                ],
                output="screen",
            ),

            Node(
                package="soil_sampler_driver",
                executable="teros12_node",
                name="teros12",
                namespace="soil_sampler",
                output="screen",
                parameters=[config_file],
            ),

            Node(
                package="soil_sampler_control",
                executable="soil_sampler_node",
                name="soil_sampler",
                namespace="soil_sampler",
                output="screen",
                parameters=[config_file],
            ),
            
            soil_sampler_experiment_launch,
            
        ]
    )
