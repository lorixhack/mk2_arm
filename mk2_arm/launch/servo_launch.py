#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="simplified_arm_assembly", 
            package_name="mk2_arm"
        )
        .robot_description(file_path="config/simplified_arm_assembly.urdf.xacro")
        .robot_description_semantic(file_path="config/simplified_arm_assembly.srdf")
        .robot_description_kinematics(file_path="config/bio_ik_kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("mk2_arm"),
        "config",
        "moveit.rviz"
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(os.path.join(
            get_package_share_directory("mk2_arm"),
            "config",
            "servo_config.yaml"
        ))
        .to_dict()    
    }

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "mk2_arm"}

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory("mk2_arm"),
                "config",
                "ros2_controllers.yaml"
            ),
            moveit_config.robot_description
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    mk2_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mk2_arm_controller", "-c", "/controller_manager"],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
        output="screen",
        respawn=False
    )

    return LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            mk2_arm_controller_spawner,
            servo_node,
            robot_state_publisher,
            static_tf,
        ]
    )

