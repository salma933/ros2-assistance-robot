#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_name = 'my_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # ============================================================
    # FILES
    # ============================================================

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'assistance_robot.urdf.xacro'
    )

    camera_bridge_config = os.path.join(
        pkg_share,
        'config',
        'camera_bridge.yaml'
    )

    controllers_config = os.path.join(
        pkg_share,
        'config',
        'ros2_controllers.yaml'
    )

    # ============================================================
    # ROBOT DESCRIPTION
    # ============================================================

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    robot_description,
                    value_type=str
                )
            }
        ]
    )

    # ============================================================
    # GAZEBO HARMONIC
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # ============================================================
    # SPAWN ROBOT
    # ============================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'assistance_robot',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.25'
        ],
        output='screen'
    )

    spawn_robot_delayed = TimerAction(
        period=4.0,
        actions=[
            spawn_robot
        ]
    )

    # ============================================================
    # ROS-GZ CAMERA BRIDGE
    # ============================================================

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': camera_bridge_config
            }
        ],
        output='screen'
    )

    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            controllers_config,
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    joint_state_broadcaster_delayed = TimerAction(
        period=7.0,
        actions=[
            joint_state_broadcaster
        ]
    )

    # ============================================================
    # ARM CONTROLLER
    # ============================================================

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',
            controllers_config,
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    arm_controller_delayed = TimerAction(
        period=8.0,
        actions=[
            arm_controller
        ]
    )

    # ============================================================
    # RVIZ
    # ============================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
        ros_gz_bridge,
        joint_state_broadcaster_delayed,
        arm_controller_delayed,
        rviz
    ])