from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='woosh_nav_feedback',
            executable='exec_task_feedback_node',
            name='exec_task_feedback_node',
            output='screen',
            parameters=[{
                'action_name': 'woosh_robot/robot/ExecTask',
                'operation_state_topic': 'woosh_robot/robot/OperationState',
                'go_mark_topic': '/navigation/go_mark',
                'feedback_topic': '/navigation/feedback',
                'startup_mark_no': '',
                'wait_for_taskable': True,
                'server_wait_sec': 20.0,
                'task_timeout_sec': 120.0,
                'task_type': 1,
                'direction': 0,
                'task_type_no': 0,
            }],
        )
    ])
