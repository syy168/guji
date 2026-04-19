#!/usr/bin/env python3
import json
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from woosh_robot_msgs.action import ExecTask
from woosh_robot_msgs.msg import OperationState, OperationStateRobotBit
from woosh_task_msgs.msg import State


class ExecTaskFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__('exec_task_feedback_node')

        self.declare_parameter('action_name', '/woosh_robot/robot/ExecTask')
        self.declare_parameter('operation_state_topic', 'woosh_robot/robot/OperationState')
        self.declare_parameter('go_mark_topic', '/navigation/go_mark')
        self.declare_parameter('feedback_topic', '/navigation/feedback')
        self.declare_parameter('startup_mark_no', '')
        self.declare_parameter('wait_for_taskable', True)
        self.declare_parameter('server_wait_sec', 20.0)
        self.declare_parameter('task_timeout_sec', 120.0)
        self.declare_parameter('task_type', 1)
        self.declare_parameter('direction', 0)
        self.declare_parameter('task_type_no', 0)

        action_name = str(self.get_parameter('action_name').value).strip()
        if not action_name:
            action_name = '/woosh_robot/robot/ExecTask'
        operation_state_topic = self.get_parameter('operation_state_topic').value
        go_mark_topic = self.get_parameter('go_mark_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.wait_for_taskable = bool(self.get_parameter('wait_for_taskable').value)
        self.server_wait_sec = float(self.get_parameter('server_wait_sec').value)
        self.task_timeout_sec = float(self.get_parameter('task_timeout_sec').value)

        self.task_type = int(self.get_parameter('task_type').value)
        self.direction = int(self.get_parameter('direction').value)
        self.task_type_no = int(self.get_parameter('task_type_no').value)

        self._is_taskable = False
        self._goal_active = False
        self._goal_handle = None
        self._timeout_timer = None
        self._startup_timer = None

        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.create_subscription(String, go_mark_topic, self._on_go_mark, 10)
        self.create_subscription(OperationState, operation_state_topic, self._on_operation_state, 10)

        self.exec_task_client = ActionClient(self, ExecTask, action_name)

        startup_mark_no = str(self.get_parameter('startup_mark_no').value).strip()
        if startup_mark_no:
            self._startup_timer = self.create_timer(2.0, lambda: self._startup_once(startup_mark_no))

        self.get_logger().info('exec_task_feedback_node ready.')
        self.get_logger().info(f'action name: {action_name}')
        self.get_logger().info(f'listen go mark topic: {go_mark_topic}')
        self.get_logger().info(f'publish feedback topic: {self.feedback_topic}')

    def _startup_once(self, mark_no: str) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        self._send_goal(mark_no)

    def _publish_feedback(self, stage: str, mark_no: str, detail: str, ok: bool) -> None:
        payload = {
            'stage': stage,
            'mark_no': mark_no,
            'ok': ok,
            'detail': detail,
            'taskable': self._is_taskable,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.feedback_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _on_operation_state(self, msg: OperationState) -> None:
        self._is_taskable = bool(msg.robot & OperationStateRobotBit.K_TASKABLE)

    def _on_go_mark(self, msg: String) -> None:
        mark_no = msg.data.strip()
        if not mark_no:
            self._publish_feedback('START_REQUEST', '', 'empty mark_no', False)
            return
        self._send_goal(mark_no)

    def _send_goal(self, mark_no: str) -> None:
        if self._goal_active:
            self._publish_feedback('START_REQUEST', mark_no, 'another task is running', False)
            return

        if self.wait_for_taskable and not self._is_taskable:
            self._publish_feedback('START_REQUEST', mark_no, 'robot is not taskable now', False)
            return

        self._publish_feedback('START_REQUEST', mark_no, 'sending goal', True)

        if not self.exec_task_client.wait_for_server(timeout_sec=self.server_wait_sec):
            self._publish_feedback('START_CONFIRMED', mark_no, 'action server unavailable', False)
            return

        goal_msg = ExecTask.Goal()
        goal_msg.arg.task_id = int(self.get_clock().now().nanoseconds / 1e9)
        goal_msg.arg.type.value = int(self.task_type)
        goal_msg.arg.direction.value = int(self.direction)
        goal_msg.arg.task_type_no = int(self.task_type_no)
        goal_msg.arg.mark_no = mark_no

        self._goal_active = True
        self._goal_handle = None

        future = self.exec_task_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(lambda f: self._goal_response_cb(f, mark_no))

    def _goal_response_cb(self, future, mark_no: str) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._goal_active = False
            self._publish_feedback('START_CONFIRMED', mark_no, 'goal rejected', False)
            return

        self._goal_handle = goal_handle
        self._publish_feedback('START_CONFIRMED', mark_no, 'goal accepted', True)

        if self.task_timeout_sec > 0.0:
            self._timeout_timer = self.create_timer(self.task_timeout_sec, lambda: self._on_timeout(mark_no))

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, mark_no))

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback.fb
        detail = (
            f'dest={fb.dest}, action_type={fb.action.type.value}, '
            f'action_state={fb.action.state.value}, task_state={fb.state.value}'
        )
        self._publish_feedback('IN_PROGRESS', fb.dest, detail, True)

    def _on_timeout(self, mark_no: str) -> None:
        if not self._goal_active or self._goal_handle is None:
            return
        self._publish_feedback('TIMEOUT', mark_no, f'exceed {self.task_timeout_sec} seconds', False)
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.get_logger().warn('cancel requested for timeout'))
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _result_cb(self, future, mark_no: str) -> None:
        wrapped_result = future.result()
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        self._goal_active = False
        self._goal_handle = None

        if wrapped_result is None or wrapped_result.result is None:
            self._publish_feedback('END_CONFIRMED', mark_no, 'no result received', False)
            return

        task_proc = wrapped_result.result.ret
        state_value = task_proc.state.value

        if state_value == State.K_COMPLETED:
            self._publish_feedback('END_CONFIRMED', mark_no, 'task completed', True)
        elif state_value == State.K_CANCELED:
            self._publish_feedback('END_CONFIRMED', mark_no, 'task canceled', False)
        elif state_value == State.K_FAILED:
            self._publish_feedback('END_CONFIRMED', mark_no, 'task failed', False)
        else:
            self._publish_feedback('END_CONFIRMED', mark_no, f'unknown state: {state_value}', False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecTaskFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
