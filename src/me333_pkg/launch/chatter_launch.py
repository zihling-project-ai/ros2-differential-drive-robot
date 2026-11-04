
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_talker',
            namespace='student_six',
            remappings=[
                ('chatter', 'my_chatter')
            ]
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='my_listener',
            namespace='student_six',
            remappings=[
                ('chatter', 'my_chatter')
            ]
        ),
    ])
