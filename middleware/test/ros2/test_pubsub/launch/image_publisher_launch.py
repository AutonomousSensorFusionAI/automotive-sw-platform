from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hz = LaunchConfiguration("hz")

    hz_arg = DeclareLaunchArgument("hz", default_value="1000")

    return LaunchDescription(
        [
            hz_arg,
            Node(
                package="test_pubsub",
                executable="pub_image",
                name="test_topic_publisher",
                output="screen",
                emulate_tty=True,
                parameters=[{"hz": hz}],
            ),
        ]
    )
