# real_robot_pc_min.launch.py (ROS 2 Humble)
# Modes:
#   - autonomous:=true  -> localization SLAM + full Nav2 stack (no joystick)
#   - autonomous:=false -> localization SLAM + joystick teleop (no Nav2 stack)

import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    MAP_YAML   = "/home/alain/bumperbot_ws/map/playground_post_3_hd.yaml"
    GRAPH_BASE = "/home/alain/bumperbot_ws/map/playground_post_3_hd"
    SCAN_TOPIC = "/scan"
    BASE_FRAME = "base_footprint"
    ODOM_FRAME = "odom"

    NAV2_PARAMS = "/home/alain/bumperbot_ws/src/bumperbot_bringup/config/nav2_params.yaml"

    if not os.path.exists(NAV2_PARAMS):
        print(f"[ERROR] Nav2 params file not found: {NAV2_PARAMS}")
    else:
        print(f"[INFO] Using Nav2 params: {NAV2_PARAMS}")

    autonomous = LaunchConfiguration("autonomous").perform(context).lower() in ("true","1","yes")

    actions = []

    # --- map_server (lifecycle) + lifecycle manager (auto-activate)
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": MAP_YAML,
            "frame_id": "map",
            "use_sim_time": False,
        }],
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "bond_timeout": 0.0,
            "use_sim_time": False,
            "node_names": ["map_server"],
        }],
    )

    actions += [map_server, lifecycle_manager_map]

    # =========================
    # NORMAL/AUTONOMOUS (real robot baseline)
    # =========================

    # --- Robot State Publisher (publishes TF from your URDF fixed joints) ---
    URDF_PATH = "/home/alain/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf"
    with open(URDF_PATH, "r") as f:
        robot_description_xml = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_xml,
            "publish_frequency": 30.0,
            "use_sim_time": False,  # Critical!
        }],
        # Add this to force proper time initialization:
        ros_arguments=[
            '--log-level', 'warn',
        ],
    )

    robot_state_publisher_delayed = TimerAction(period=2.0, actions=[robot_state_publisher])
    actions += [robot_state_publisher_delayed]

    # --- slam_toolbox in localization mode ---
    slam_loc = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "mode": "localization",
            "map_file_name": GRAPH_BASE,      # serialized posegraph (base name)
            "scan_topic": SCAN_TOPIC,
            "map_frame": "map",
            "odom_frame": ODOM_FRAME,
            "base_frame": BASE_FRAME,
            "publish_tf": True,               # publishes map->odom
            "provide_odom_frame": False,
            "map_start_pose": [0.0, 0.0, 0.0]
        }],
        remappings=[("map", "/slam_map")],
        respawn=True,
        respawn_delay=2.0,
    )

    # Initial pose (published twice with a delay)
    initialpose_msg = """{
      header: {frame_id: 'map'},
      pose: { pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.05,0,0,0,0,0,
                   0,0.05,0,0,0,0,
                   0,0,0.01,0,0,0,
                   0,0,0,0.01,0,0,
                   0,0,0,0,0.01,0,
                   0,0,0,0,0,0.1] }
    }"""

    init_pose_1 = ExecuteProcess(
        cmd=["ros2","topic","pub","--once","/initialpose",
             "geometry_msgs/PoseWithCovarianceStamped", initialpose_msg],
        output="screen"
    )
    init_pose_2 = ExecuteProcess(
        cmd=["ros2","topic","pub","--once","/initialpose",
             "geometry_msgs/PoseWithCovarianceStamped", initialpose_msg],
        output="screen"
    )

    init_pose_1_delayed = TimerAction(period=3.0, actions=[init_pose_1])
    init_pose_2_delayed = TimerAction(period=6.0, actions=[init_pose_2])

    # Visualization helpers
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
    tf_path_node_delayed = TimerAction(period=7.0, actions=[tf_path_node])

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
    odom_path_node_delayed = TimerAction(period=7.0, actions=[odom_path_node])

    waypoint_markers_node = Node(
            package='bumperbot_viz_tools',
            executable='waypoint_markers_node.py',
            name='waypoint_markers',
            output='screen',
            parameters=[{
                # You can add parameters here if needed
            }]
        )
    
    waypoint_markers_node_delayed = TimerAction(period=7.0, actions=[waypoint_markers_node])

    actions += [slam_loc, init_pose_1_delayed, init_pose_2_delayed,
                tf_path_node_delayed, odom_path_node_delayed, waypoint_markers_node_delayed]

    if autonomous:
        # =========================
        # Autonomous Nav2 stack (no joystick)
        # =========================
        planner_server = Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
            parameters=[NAV2_PARAMS],
        )

        controller_server = Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[NAV2_PARAMS],
        )

        behavior_server = Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", output="screen",
            parameters=[NAV2_PARAMS],
        )

        bt_navigator = Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
            parameters=[NAV2_PARAMS],
        )

        smoother_server = Node(
            package="nav2_smoother", executable="smoother_server",
            name="smoother_server", output="screen",
            parameters=[NAV2_PARAMS],
        )

        lifecycle_manager_nav2 = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav2",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                    "smoother_server",
                ],
            }],
        )

        nav_twist_stamper = Node(
            package="bumperbot_twist_stamper",
            executable="twist_stamper_node",
            name="nav_twist_stamper",
            parameters=[{
                "in_topic": "/cmd_vel",  # Nav2 publishes here
                "out_topic": "/diff_drive_controller/cmd_vel",
                "frame_id": "base_footprint",
            }],
        )

        nav2_group = GroupAction([
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            smoother_server,
            lifecycle_manager_nav2,
            nav_twist_stamper,
        ])

        # Delay Nav2 bringup to let SLAM publish map->odom first
        nav2_group_delayed = TimerAction(period=10.0, actions=[nav2_group])
        actions += [nav2_group_delayed]

        # (Optional) Example auto-goal could be added here if desired

    else:
        # --- Manual mode (joystick teleop), Nav2 controller is NOT launched ---
        joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("bumperbot_controller"),
                    "launch",
                    "joystick_teleop.launch.py"
                )
            )
        )
        actions += [joystick]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "autonomous",
            default_value="true",
            description=(
                "If true: localization + Nav2 autonomous navigation. "
                "If false: localization + joystick teleop."
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
