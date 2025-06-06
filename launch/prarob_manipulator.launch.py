# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    robot_name = "prarob_manipulator"
    package_name = "ros2_prarob"
    rviz_config = os.path.join(get_package_share_directory(
        package_name), "launch", "prarob_manipulator.rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "controllers.yaml"
    )

    camera_config_path = os.path.join(
        get_package_share_directory('prarob_calib'),
        'config',
        'camera_params.yaml'
        )

    yolo_launch_file = os.path.join(
        get_package_share_directory('yolo_bringup'),
        'launch',
        'yolo.launch.py',
    )

        
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch_file)
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            #output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',      # translation: x y z
                '0', '0', '0', '1', # rotation: qx qy qz qw
                'world',            # parent frame
                'link1'             # child frame
            ],
        ),

        Node(                       # ros2 launch usb_cam camera.launch.py for computer camera
            package='usb_cam', 
            executable='usb_cam_node_exe',
            name="camera",
            parameters=[camera_config_path],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ]
        ),

        Node(
            package='ros2_prarob',
            executable='kinematics.py',
            name='kinematics_node',
            output='screen'
        ),

        Node(
            package='ros2_prarob',
            executable='path_planner.py',
            name='path_planner_node',
            output='screen'
        ),

        Node(
            package='ros2_prarob',
            executable='move_robot.py',
            name='prarob_client_node',
            output='screen'
        ),

    ])