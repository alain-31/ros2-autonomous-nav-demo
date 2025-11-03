import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    urdf_path = "/home/raspberry/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf"
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "bringup_cli_like.launch.py"
        ),
    )
    
   
    # Run the RPLIDAR C1 driver on the Pi
    lidar = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sllidar_ros2"),
            "launch",
            "sllidar_c1_launch.py"
        ),
        launch_arguments={
            "serial_port": "/dev/lidar-c1",
            "serial_baudrate": "460800",
            "frame_id": "laser_link",
            #"motor_pwm": "200",           # adjust up/down to land near 5 Hz
            #"angle_compensate": "false",
        }.items(),
    )

    #  robot_state_publisher publishes ~/robot_description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

        # Throttle /scan -> /scan_5hz (BEST_EFFORT, depth=1)
    scan_throttle = Node(
        package="bumperbot_throttle",
        executable="scan_throttle",
        name="scan_throttle",
        output="screen",
        parameters=[{
            "input_topic": "/scan",
            "output_topic": "/scan_2hz",
            "rate_hz": 2.0,                 # adjust if you want 4/6 Hz, etc.
            "reliability": "best_effort",   # or "reliable" on wired links
            "use_msg_stamp": True,          # throttle by message timestamp
        }],
    )

    return LaunchDescription([
        lidar,
        #scan_throttle,
        #rsp,
        controller,
    ])  