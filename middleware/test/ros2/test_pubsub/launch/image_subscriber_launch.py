from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="test_pubsub",
                executable="sub_image",
                name="test_topic_subsriber",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
