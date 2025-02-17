from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros

def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "name:=ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "tf_prefix:=", tf_prefix,
            " ",
            "simulation_controllers:=", controllers_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_2f_85_controller", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(activate_joint_controller),
    )

    moveit_config = MoveItConfigsBuilder("ur", package_name="ur5e_moveit_config").to_moveit_configs()
    moveit_demo = generate_demo_launch(moveit_config)

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        initial_joint_controller_spawner,
        moveit_demo,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", description="Tipo de UR", default_value="ur5e"),
        DeclareLaunchArgument("description_file",
                              default_value=PathJoinSubstitution([FindPackageShare("ur5e_description"), "urdf/ur5e", "ur.urdf.xacro"]),
                              description="URDF/XACRO del robot."),
        DeclareLaunchArgument("controllers_file",
                              default_value=PathJoinSubstitution([FindPackageShare("ur5e_description"), "config", "ur_controllers.yaml"]),
                              description="Configuración de controladores."),
        DeclareLaunchArgument("tf_prefix", default_value='""', description="Prefijo de transformaciones."),
        DeclareLaunchArgument("activate_joint_controller", default_value="true", description="Activar controlador de juntas."),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller", description="Controlador inicial."),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Lanzar RViz."),
        DeclareLaunchArgument("rviz_config_file",
                              default_value=PathJoinSubstitution([FindPackageShare("ur5e_description"), "rviz", "view_robot.rviz"]),
                              description="Archivo de configuración de RViz."),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])