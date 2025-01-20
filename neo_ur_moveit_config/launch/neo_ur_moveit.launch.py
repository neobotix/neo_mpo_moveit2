# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue, ParameterFile

from neo_ur_moveit_config.launch_common import load_yaml

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    my_robot = LaunchConfiguration("my_robot")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    gripper_type = LaunchConfiguration("gripper")

    # General arguments
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
   
    gazebo_controllers = context.perform_substitution(use_gazebo)
    if (gazebo_controllers == True):
        urdf = os.path.join(get_package_share_directory("neo_simulation2"),
            'robots', 
            str(my_robot.perform(context)), 
            str(my_robot.perform(context)) + '.urdf.xacro')
    else:
        urdf = os.path.join(get_package_share_directory('neo_mpo_700-2'),
            'robot_model/mpo_700',
            'mpo_700.urdf.xacro')

    robot_description_content = Command([
        "xacro", " ", urdf, " ", 
        'arm_type:=', ur_type, " ",
        'use_gazebo:=', gazebo_controllers, " ",
        'gripper_type:=', gripper_type.perform(context), " ",
        'robot_prefix:=', my_robot,
        ])

    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    srdf = os.path.join(get_package_share_directory(str(moveit_config_package.perform(context))),
        'srdf',
        str(my_robot.perform(context)) + '.srdf.xacro'
    )

    robot_description_semantic_content = ParameterValue(
        Command([
            "xacro", " ", srdf, " ", 'prefix:=',
            prefix, " ", 'gripper_type:=',
            gripper_type
            ]),
        value_type=str)

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content} 

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    joint_limits_yaml =os.path.join(
        get_package_share_directory(str(moveit_config_package.perform(context))),
        "config",
        str(moveit_joint_limits_file.perform(context))
    )

    joint_limits_yaml_with_substitutions = ParameterFile(joint_limits_yaml, allow_substs=True)
    # Evaluate the parameter file to apply dynamic substitutions
    joint_limits_yaml_with_substitutions.evaluate(context)
    # Load the YAML file with the applied substitutions
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", 
                        str(joint_limits_yaml_with_substitutions.param_file))
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization 
            default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds 
            default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1
        }
    }
    ompl_planning_yaml = load_yaml(str(moveit_config_package.perform(context)), "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = os.path.join(
        get_package_share_directory(str(moveit_config_package.perform(context))),
        "config",
        "controllers.yaml",
    )

    controllers_yaml_with_substitutions = ParameterFile(controllers_yaml, allow_substs=True)
    # Evaluate the parameter file to apply dynamic substitutions
    controllers_yaml_with_substitutions.evaluate(context)
    # Load the YAML file with the applied substitutions    
    controllers_yaml_with_substitutions = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", 
                    str(controllers_yaml_with_substitutions.param_file)),
    )
    # the scaled_joint_trajectory_controller does not work on fake hardware
    if gazebo_controllers == "true":
        controllers_yaml_with_substitutions["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml_with_substitutions["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml_with_substitutions,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True,
        "publish_robot_description_semantic":True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            # robot_description_planning,
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'my_robot', 
            default_value='mpo_700',
            description='Neobotix Robot Types: "mpo_700", "mpo_500"'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper', 
            default_value='',
            description='Gripper if needed. For now, just Robotiq gripper: 2f_140'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="neo_ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz",
        default_value="true",
        description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_gazebo",
        default_value="false",
        description="Using Gazebo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

