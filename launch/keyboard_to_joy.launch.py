import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_to_joy_node = Node(
        package='keyboard_to_joy', name='keyboard_to_joy', output='screen', executable='keyboard_to_joy',
        parameters=[{'exit_on_esc': True}], arguments=['__log_level:=warn'])
    return LaunchDescription([
        keyboard_to_joy_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=keyboard_to_joy_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])
