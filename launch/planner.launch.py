import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    geojson_file_path = LaunchConfiguration("geojson_file_path").perform(context)
    geotiff_file_path = LaunchConfiguration("geotiff_file_path").perform(context)

    param_file = os.path.join(
        get_package_share_directory("farmbot_planner"), "config", "params.yaml"
    )

    nodes_array = []

    cartograph = Node(
        package="farmbot_planner",
        executable="cartograph",
        name="cartograph",
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
            {"geojson_file_path": geojson_file_path},
            {"geotiff_file_path": geotiff_file_path},
        ],
    )
    nodes_array.append(cartograph)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="fbot")
    geojson_file_path = DeclareLaunchArgument(
        "geojson_file_path", default_value="./wur.json"
    )
    geotiff_file_path = DeclareLaunchArgument(
        "geotiff_file_path", default_value="./wur.tiff"
    )

    return LaunchDescription(
        [
            namespace_arg,
            geojson_file_path,
            geotiff_file_path,
            OpaqueFunction(function=launch_setup),
        ]
    )
