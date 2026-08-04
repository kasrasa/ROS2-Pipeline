import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from robot_state_interfaces.action import MoveToJointPosition


class MoveActionServer(Node):
    SUPPORTED_TASKS = {
        'inspection',
        'maintenance',
        'calibration',
    }

    JOINT_LIMIT = 3.14

    def __init__(self) -> None:
        super().__init__('move_action_server')

        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()
        self.goal_active = False

        action_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.command_publisher = self.create_publisher(
            JointState,
            '/commanded_joint_states',
            action_state_qos,
        )

        self.protective_stop = False

        self.protective_stop_subscriber = self.create_subscription(
            Bool,
            '/protective_stop',
            self.protective_stop_callback,
            10,
            callback_group=self.callback_group,
        )

        self.joint_names = [
            'shoulder_joint',
            'elbow_joint',
        ]

        self.current_shoulder = 0.0
        self.current_elbow = 0.0

        self.action_server = ActionServer(
            self,
            MoveToJointPosition,
            '/move_to_joint_position',
            execute_callback=self.execute_move,
            goal_callback=self.validate_goal,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Move Action Server started and ready to accept goals.'
        )

    def protective_stop_callback(self, msg: Bool) -> None:
        previous_state = self.protective_stop
        self.protective_stop = bool(msg.data)

        if self.protective_stop and not previous_state:
            self.get_logger().error(
                'Protective stop activated. Active movement will be aborted.'
            )
        elif previous_state and not self.protective_stop:
            self.get_logger().info(
                'Protective stop cleared.'
            )

    def publish_commanded_state(
        self,
        shoulder_position: float,
        elbow_position: float,
        shoulder_velocity: float,
        elbow_velocity: float,
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names.copy()
        msg.position = [
            float(shoulder_position),
            float(elbow_position),
        ]
        msg.velocity = [
            float(shoulder_velocity),
            float(elbow_velocity),
        ]
        msg.effort = []

        self.command_publisher.publish(msg)

    def publish_feedback(
        self,
        goal_handle,
        progress: float,
        phase: str,
    ) -> None:
        feedback = MoveToJointPosition.Feedback()
        feedback.progress = float(progress)
        feedback.phase = phase

        with self.state_lock:
            feedback.current_shoulder = self.current_shoulder
            feedback.current_elbow = self.current_elbow

        goal_handle.publish_feedback(feedback)

    def create_result(
        self,
        success: bool,
        message: str,
    ) -> MoveToJointPosition.Result:
        result = MoveToJointPosition.Result()
        result.success = success
        result.message = message

        with self.state_lock:
            result.final_shoulder = self.current_shoulder
            result.final_elbow = self.current_elbow

        return result

    def validate_goal(
        self,
        goal_request: MoveToJointPosition.Goal,
    ) -> GoalResponse:
        self.get_logger().info(
            'Received goal request: '
            f'task={goal_request.task_name}, '
            f'shoulder={goal_request.target_shoulder:.3f}, '
            f'elbow={goal_request.target_elbow:.3f}, '
            f'duration={goal_request.duration_sec:.3f}s, '
            f'timeout={goal_request.timeout_sec:.3f}s'
        )

        task_name = goal_request.task_name.strip().lower()

        if task_name not in self.SUPPORTED_TASKS:
            self.get_logger().warning(
                'Goal rejected: unsupported task name.'
            )
            return GoalResponse.REJECT

        if not (
            -self.JOINT_LIMIT
            <= goal_request.target_shoulder
            <= self.JOINT_LIMIT
        ):
            self.get_logger().warning(
                'Goal rejected: shoulder target is outside joint limits.'
            )
            return GoalResponse.REJECT

        if not (
            -self.JOINT_LIMIT
            <= goal_request.target_elbow
            <= self.JOINT_LIMIT
        ):
            self.get_logger().warning(
                'Goal rejected: elbow target is outside joint limits.'
            )
            return GoalResponse.REJECT

        if goal_request.duration_sec <= 0.0:
            self.get_logger().warning(
                'Goal rejected: duration_sec must be positive.'
            )
            return GoalResponse.REJECT

        if goal_request.duration_sec > 30.0:
            self.get_logger().warning(
                'Goal rejected: duration_sec must not exceed 30 seconds.'
            )
            return GoalResponse.REJECT

        if goal_request.timeout_sec < 0.0:
            self.get_logger().warning(
                'Goal rejected: timeout_sec must be non-negative.'
            )
            return GoalResponse.REJECT

        if self.protective_stop:
            self.get_logger().warning(
                'Goal rejected: protective stop is active.'
            )
            return GoalResponse.REJECT

        with self.state_lock:
            if self.goal_active:
                self.get_logger().warning(
                    'Goal rejected: another goal is already active.'
                )
                return GoalResponse.REJECT

            self.goal_active = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info(
            f'Cancellation requested for goal {goal_handle.goal_id}.'
        )
        return CancelResponse.ACCEPT

    def check_interruption(
        self,
        goal_handle,
        execution_start_time: float,
        timeout_sec: float,
    ) -> MoveToJointPosition.Result | None:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()

            with self.state_lock:
                shoulder = self.current_shoulder
                elbow = self.current_elbow

            self.publish_commanded_state(
                shoulder,
                elbow,
                0.0,
                0.0,
            )

            self.get_logger().warning(
                'Goal canceled by client.'
            )

            return self.create_result(
                success=False,
                message='Task canceled by client.',
            )

        if self.protective_stop:
            goal_handle.abort()

            with self.state_lock:
                shoulder = self.current_shoulder
                elbow = self.current_elbow

            self.publish_commanded_state(
                shoulder,
                elbow,
                0.0,
                0.0,
            )

            self.get_logger().error(
                'Goal aborted because protective stop became active.'
            )

            return self.create_result(
                success=False,
                message='Task aborted due to protective stop.',
            )

        elapsed_seconds = time.monotonic() - execution_start_time

        if timeout_sec > 0.0 and elapsed_seconds > timeout_sec:
            goal_handle.abort()

            with self.state_lock:
                shoulder = self.current_shoulder
                elbow = self.current_elbow

            self.publish_commanded_state(
                shoulder,
                elbow,
                0.0,
                0.0,
            )

            self.get_logger().error(
                f'Goal timed out after {elapsed_seconds:.2f} seconds.'
            )

            return self.create_result(
                success=False,
                message=(
                    f'Task timed out after '
                    f'{elapsed_seconds:.2f} seconds.'
                ),
            )

        return None

    def sleep_with_interruption_checks(
        self,
        duration_sec: float,
        goal_handle,
        execution_start_time: float,
        timeout_sec: float,
        check_period_sec: float = 0.05,
    ) -> MoveToJointPosition.Result | None:
        sleep_start = time.monotonic()

        while time.monotonic() - sleep_start < duration_sec:
            interruption_result = self.check_interruption(
                goal_handle,
                execution_start_time,
                timeout_sec,
            )

            if interruption_result is not None:
                return interruption_result

            remaining = duration_sec - (
                time.monotonic() - sleep_start
            )

            time.sleep(
                min(check_period_sec, max(0.0, remaining))
            )

        return None

    def execute_move(
        self,
        goal_handle,
    ) -> MoveToJointPosition.Result:
        goal = goal_handle.request
        execution_start_time = time.monotonic()

        self.get_logger().info(
            f'Executing task "{goal.task_name}".'
        )

        try:
            with self.state_lock:
                start_shoulder = self.current_shoulder
                start_elbow = self.current_elbow

            self.publish_feedback(
                goal_handle,
                progress=0.0,
                phase='VALIDATING',
            )

            interruption_result = (
                self.sleep_with_interruption_checks(
                    duration_sec=0.2,
                    goal_handle=goal_handle,
                    execution_start_time=execution_start_time,
                    timeout_sec=goal.timeout_sec,
                )
            )

            if interruption_result is not None:
                return interruption_result

            update_rate_hz = 10.0
            step_period = 1.0 / update_rate_hz
            total_steps = max(
                1,
                int(goal.duration_sec * update_rate_hz),
            )

            shoulder_velocity = (
                goal.target_shoulder - start_shoulder
            ) / goal.duration_sec

            elbow_velocity = (
                goal.target_elbow - start_elbow
            ) / goal.duration_sec

            for step in range(1, total_steps + 1):
                interruption_result = self.check_interruption(
                    goal_handle,
                    execution_start_time,
                    goal.timeout_sec,
                )

                if interruption_result is not None:
                    return interruption_result

                movement_progress = step / total_steps

                shoulder_position = (
                    start_shoulder
                    + movement_progress
                    * (
                        goal.target_shoulder
                        - start_shoulder
                    )
                )

                elbow_position = (
                    start_elbow
                    + movement_progress
                    * (
                        goal.target_elbow
                        - start_elbow
                    )
                )

                with self.state_lock:
                    self.current_shoulder = shoulder_position
                    self.current_elbow = elbow_position

                self.publish_commanded_state(
                    shoulder_position,
                    elbow_position,
                    shoulder_velocity,
                    elbow_velocity,
                )

                self.publish_feedback(
                    goal_handle,
                    progress=0.05 + movement_progress * 0.70,
                    phase='MOVING',
                )

                interruption_result = (
                    self.sleep_with_interruption_checks(
                        duration_sec=step_period,
                        goal_handle=goal_handle,
                        execution_start_time=execution_start_time,
                        timeout_sec=goal.timeout_sec,
                    )
                )

                if interruption_result is not None:
                    return interruption_result

            with self.state_lock:
                self.current_shoulder = goal.target_shoulder
                self.current_elbow = goal.target_elbow

            self.publish_commanded_state(
                goal.target_shoulder,
                goal.target_elbow,
                0.0,
                0.0,
            )

            self.publish_feedback(
                goal_handle,
                progress=0.80,
                phase='AT_TARGET',
            )

            interruption_result = (
                self.sleep_with_interruption_checks(
                    duration_sec=0.2,
                    goal_handle=goal_handle,
                    execution_start_time=execution_start_time,
                    timeout_sec=goal.timeout_sec,
                )
            )

            if interruption_result is not None:
                return interruption_result

            capture_steps = 10

            for step in range(1, capture_steps + 1):
                interruption_result = self.check_interruption(
                    goal_handle,
                    execution_start_time,
                    goal.timeout_sec,
                )

                if interruption_result is not None:
                    return interruption_result

                capture_progress = step / capture_steps

                self.publish_feedback(
                    goal_handle,
                    progress=0.80 + capture_progress * 0.19,
                    phase='CAPTURING',
                )

                interruption_result = (
                    self.sleep_with_interruption_checks(
                        duration_sec=0.05,
                        goal_handle=goal_handle,
                        execution_start_time=execution_start_time,
                        timeout_sec=goal.timeout_sec,
                    )
                )

                if interruption_result is not None:
                    return interruption_result

            self.publish_feedback(
                goal_handle,
                progress=1.0,
                phase='COMPLETED',
            )

            goal_handle.succeed()

            self.get_logger().info(
                f'Task "{goal.task_name}" completed successfully.'
            )

            return self.create_result(
                success=True,
                message='Goal completed successfully.',
            )

        except Exception as error:
            self.get_logger().exception(
                f'Unexpected action execution error: {error}'
            )

            if goal_handle.is_active:
                goal_handle.abort()

            with self.state_lock:
                shoulder = self.current_shoulder
                elbow = self.current_elbow

            self.publish_commanded_state(
                shoulder,
                elbow,
                0.0,
                0.0,
            )

            return self.create_result(
                success=False,
                message=f'Goal aborted: {error}',
            )

        finally:
            with self.state_lock:
                self.goal_active = False

    def destroy_node(self) -> None:
        self.action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MoveActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()