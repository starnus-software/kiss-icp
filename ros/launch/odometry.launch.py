# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare("kiss_icp")
    return LaunchDescription(
        [
            # ROS 2 parameters
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_cyclonedds_cpp",
                description="The RMW implementation to use (e.g., rmw_cyclonedds_cpp).",
            ),
            DeclareLaunchArgument(
                "topic",
                default_value="/kiss_icp_merged_cloud",
                description="sensor_msg/PointCloud2 topic to process",
            ),
            DeclareLaunchArgument("bagfile", default_value=""),
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("odom_frame", default_value="kiss_icp_odom"),
            DeclareLaunchArgument("map_frame", default_value="kiss_icp_map"),
            DeclareLaunchArgument(
                "base_frame", default_value="kiss_icp_frame_base_link"
            ),
            DeclareLaunchArgument("publish_odom_tf", default_value="publish_odom_tf"),
            # KISS-ICP parameters
            DeclareLaunchArgument("deskew", default_value="false"),
            DeclareLaunchArgument("max_range", default_value="30.0"),
            DeclareLaunchArgument("min_range", default_value="0.5"),
            # This thing is still not suported: https://github.com/ros2/launch/issues/290#issuecomment-1438476902
            #  DeclareLaunchArgument("voxel_size", default_value=None),
            Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=[
                    "-d",
                    PathJoinSubstitution([current_pkg, "rviz", "kiss_icp_ros2.rviz"]),
                ],
                condition=IfCondition(LaunchConfiguration("visualize")),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])
                ),
            ),
        ]
    )
