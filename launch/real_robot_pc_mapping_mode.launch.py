# mapping_mode.launch.py (ROS 2 Humble)
# Pure mapping mode - drive with joystick to create the map

import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Configuration paths
    SLAM_PARAMS = "/home/alain/bumperbot_ws/src/bumperbot_slam_toolbox/config/slam_params.yaml"
    URDF_PATH = "/home/alain/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf"
    
    # Frames
    SCAN_TOPIC = "/scan"
    BASE_FRAME = "base_footprint"
    ODOM_FRAME = "odom"
    
    # --- Robot State Publisher (publishes TF from URDF fixed joints) ---
    with open(URDF_PATH, "r") as f:
        robot_description_xml = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_xml,
            "publish_frequency": 30.0
        }],
    )

    # --- SLAM Toolbox (MAPPING MODE) ---
    slam_mapping = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            SLAM_PARAMS,
        ],
    )

    # --- Visualization Tools ---
    tf_path_node = Node(
        package="bumperbot_viz_tools",
        executable="tf_path_node",
        name="tf_path_node",
        output="screen",
        parameters=[{
            "fixed_frame": "map",
            "base_frame": BASE_FRAME,
            "publish_rate_hz": 30.0,
            "jump_distance_m": 0.10,
            "debounce_samples": 1,
            "min_seconds_between_markers": 0.5,
            "map_odom_jump_m": 0.30,
            "map_odom_jump_rad": 0.20,
            "detect_via_map_odom": True,
        }],
    )

    odom_path_node = Node(
        package="bumperbot_viz_tools",
        executable="odom_path_node",
        name="odom_path_node",
        output="screen",
        parameters=[{
            "fixed_frame": ODOM_FRAME,
            "base_frame": BASE_FRAME,
            "publish_rate_hz": 30.0,
            "max_path_length": 20000,
            "min_step_m": 0.0,
            "min_step_rad": 0.0
        }],
    )

    # --- Joystick Teleop (REQUIRED for mapping) ---
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        )
    )

    # --- Delayed Starts ---
    slam_mapping_delayed = TimerAction(period=2.0, actions=[slam_mapping])
    tf_path_node_delayed = TimerAction(period=4.0, actions=[tf_path_node])
    odom_path_node_delayed = TimerAction(period=4.0, actions=[odom_path_node])

    return LaunchDescription([
        robot_state_publisher,
        slam_mapping_delayed,
        tf_path_node_delayed,
        odom_path_node_delayed,
        joystick,
    ])