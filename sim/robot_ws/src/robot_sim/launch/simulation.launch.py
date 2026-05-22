"""
Simulacion completa: Gazebo + Nav2 + EKF + RViz2
Lanza todo con un solo comando:
    ros2 launch robot_sim simulation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = get_package_share_directory("robot_sim")

    # ---------------------------------------------------------------
    # Argumentos
    # ---------------------------------------------------------------
    use_sim_time   = LaunchConfiguration("use_sim_time",   default="true")
    open_rviz      = LaunchConfiguration("open_rviz",      default="true")
    world_file     = LaunchConfiguration("world",          default=os.path.join(pkg, "worlds", "outdoor.world"))
    nav2_params    = LaunchConfiguration("nav2_params",    default=os.path.join(pkg, "config", "nav2_params.yaml"))
    ekf_params     = LaunchConfiguration("ekf_params",     default=os.path.join(pkg, "config", "ekf.yaml"))

    declare_args = [
        DeclareLaunchArgument("use_sim_time",  default_value="true"),
        DeclareLaunchArgument("open_rviz",     default_value="true"),
        DeclareLaunchArgument("world",         default_value=world_file),
        DeclareLaunchArgument("nav2_params",   default_value=nav2_params),
        DeclareLaunchArgument("ekf_params",    default_value=ekf_params),
    ]

    # ---------------------------------------------------------------
    # URDF via xacro
    # ---------------------------------------------------------------
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    # ---------------------------------------------------------------
    # 1. Gazebo
    # ---------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
            "pause": "false",
        }.items(),
    )

    # ---------------------------------------------------------------
    # 2. robot_state_publisher
    # ---------------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    # ---------------------------------------------------------------
    # 3. Spawn robot en Gazebo
    # ---------------------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15",
            "-Y", "0.0",
        ],
        output="screen",
    )

    # ---------------------------------------------------------------
    # 4. EKF — robot_localization
    # ---------------------------------------------------------------
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/odometry/filtered", "/odom/filtered"),
        ],
    )

    # navsat_transform: convierte GPS lat/lon → coordenadas del mapa
    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[
            ekf_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/imu/data",     "/imu/data"),
            ("/gps/fix",      "/gps/fix"),
            ("/odometry/filtered", "/odom/filtered"),
        ],
    )

    # ---------------------------------------------------------------
    # 5. Nav2 — bringup completo
    # ---------------------------------------------------------------
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    # ---------------------------------------------------------------
    # 6. RViz2
    # ---------------------------------------------------------------
    rviz_config = os.path.join(pkg, "config", "sim.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(open_rviz),
        output="screen",
    )

    # ---------------------------------------------------------------
    # Transform estatico map → odom
    # Necesario para que Nav2 tenga el frame "map" disponible.
    # Cuando se integre SLAM esto se reemplaza por el nodo de SLAM.
    # ---------------------------------------------------------------
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # ---------------------------------------------------------------
    # Secuencia con delays para dar tiempo a Gazebo de arrancar
    # ---------------------------------------------------------------
    return LaunchDescription([
        *declare_args,
        gazebo,
        rsp,
        map_to_odom_tf,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=4.0, actions=[ekf_node]),
        TimerAction(period=4.0, actions=[navsat_node]),
        TimerAction(period=5.0, actions=[nav2]),
        TimerAction(period=6.0, actions=[rviz]),
    ])
