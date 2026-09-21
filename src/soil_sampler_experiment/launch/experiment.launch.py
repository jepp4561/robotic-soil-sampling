import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_soil_sampler_experiment = get_package_share_directory(
        "soil_sampler_experiment"
    )

    config_file = os.path.join(
        pkg_soil_sampler_experiment,
        "config",
        "experiment.yaml",
    )

    experiment_node = Node(
        package="soil_sampler_experiment",
        executable="experiment_node",
        name="soil_sampler_experiment",
        output="screen",
        parameters=[config_file],
    )

    visualization_node = Node(
        package="soil_sampler_experiment",
        executable="experiment_visualization",
        name="soil_sampler_experiment_visualization",
        output="screen",
    )

    return LaunchDescription([
        experiment_node,
        visualization_node,
    ])