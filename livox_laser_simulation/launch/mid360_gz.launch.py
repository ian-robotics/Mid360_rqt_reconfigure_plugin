from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("livox_laser_simulation"))
    prefix = Path(get_package_prefix("livox_laser_simulation"))
    models = share / "models"
    plugins = prefix / "lib"

    return LaunchDescription([
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [str(models), ":", ":",
             EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            f"{plugins}:{__import__('os').environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"),
        ExecuteProcess(cmd=["gz", "sim", "-r", str(share / "worlds" / "mid360_gz.sdf")], output="screen"),

        # ROS /cmd_vel (teleop) -> gz /cmd_vel (X1 DiffDrive). "]" means the
        # bridge subscribes on the ROS side and publishes to gz.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            output="screen",
        ),

        # gz sim time -> ROS /clock, so that RViz, /tf and the point clouds
        # (all sim-stamped) share one time base.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),

        # The ground-truth TF (odom -> X1/base_link) is published by the
        # LivoxGzSystem adapter itself (see <tf_pose_topic> in models/X1),
        # because DiffDrive wheel odometry is unusable on a skid-steer
        # vehicle. Only the sensor mounting transform is bridged here.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0.0812", "--y", "0", "--z", "0.34",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "X1/base_link",
                "--child-frame-id", "livox_link",
            ],
        ),

        # Keyboard teleop in its own terminal: launch does not forward stdin
        # to child processes, so the teleop needs a dedicated window (xterm).
        # Note: xterm -e needs the command as separate argv elements.
        ExecuteProcess(
            cmd=["xterm", "-e", "bash", "-c",
                "source /opt/ros/humble/setup.bash && "
                "source " + str(prefix.parent / "setup.bash") + " && "
                "ros2 run teleop_twist_keyboard teleop_twist_keyboard"],
            output="screen"),
    ])
