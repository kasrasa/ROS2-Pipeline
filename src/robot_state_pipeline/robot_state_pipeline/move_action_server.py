import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_state_interfaces.action import MoveToJointPosition


class MoveActionServer(Node):
    def __init__(self) -> None:
        super().__init__('move_action_server')
        # Publisher for commanded joint states replacing the timer positions
        self.command_publisher = self.create_publisher(
            JointState,
            '/commanded_joint_states',
            10
        )

        self.joint_names = ['shoulder_joint', 'elbow_joint']

        self.current_shoulder = 0.0
        self.current_elbow = 0.0

        # action server for moving to joint positions
        self._action_server = ActionServer(
            self,
            MoveToJointPosition,
            'move_to_joint_position',
            execute_callback=self.execute_move,
            goal_callback=self.validate_goal,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Move Action Server has been started. Ready to accept goals.')

    def publish_commanded_state(self, shoulder_pos: float, elbow_pos: float, should_vel:float, elbow_vel:float) -> None:
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names.copy()
        msg.position = [shoulder_pos, elbow_pos]
        msg.velocity = [should_vel, elbow_vel]
        msg.effort = []

        self.command_publisher.publish(msg)

    def validate_goal(self, goal_request: MoveToJointPosition.Goal) -> GoalResponse:
        self.get_logger().info(
            "Received goal request with target positions: "
            f"shoulder: {goal_request.target_shoulder:.3f}, "
            f"elbow: {goal_request.target_elbow:.3f}"
            f"duration: {goal_request.duration_sec:.3f} seconds"
        )

        if goal_request.target_shoulder < -3.14 or goal_request.target_shoulder > 3.14:
            self.get_logger().warning("Goal rejected: target_shoulder out of range (-3.14 to 3.14).")
            return GoalResponse.REJECT
        
        if goal_request.target_elbow < -3.14 or goal_request.target_elbow > 3.14:
            self.get_logger().warning("Goal rejected: target_elbow out of range (-3.14 to 3.14).")
            return GoalResponse.REJECT

        if goal_request.duration_sec <= 0:
            self.get_logger().warning("Goal rejected: duration_sec must be positive.")
            return GoalResponse.REJECT
        
        if goal_request.duration_sec > 30:
            self.get_logger().warning("Goal rejected: duration_sec exceeds maximum allowed (30 seconds).")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info(f'Received cancel request for goal: {goal_handle}')
        return CancelResponse.ACCEPT

    def execute_move(self, goal_handle) -> MoveToJointPosition.Result:
        self.get_logger().info(f'Executing goal: {goal_handle.request}')
        goal = goal_handle.request

        update_rate_hz = 10.0
        step_period = 1.0 / update_rate_hz
        total_steps = max(1, int(goal.duration_sec * update_rate_hz))

        feedback = MoveToJointPosition.Feedback()

        start_shoulder = self.current_shoulder
        start_elbow = self.current_elbow

        shoulder_velocity = (
            goal.target_shoulder - start_shoulder
        ) / goal.duration_sec

        elbow_velocity = (
            goal.target_elbow - start_elbow
        ) / goal.duration_sec

        for step in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                self.publish_commanded_state(self.current_shoulder, self.current_elbow, 0.0, 0.0)

                result = MoveToJointPosition.Result()
                result.success = False
                result.message = 'Goal was canceled by the client.'
                result.final_shoulder = feedback.current_shoulder
                result.final_elbow = feedback.current_elbow

                self.get_logger().info('Goal canceled.')
                return result

            progress = (step + 1) / total_steps
            self.current_shoulder = start_shoulder + progress * (goal.target_shoulder - start_shoulder)
            self.current_elbow = start_elbow + progress * (goal.target_elbow - start_elbow)

            feedback.progress = progress
            feedback.current_shoulder = self.current_shoulder
            feedback.current_elbow = self.current_elbow

            goal_handle.publish_feedback(feedback)

            self.publish_commanded_state(self.current_shoulder, self.current_elbow, shoulder_velocity, elbow_velocity)

            self.get_logger().info(
                f'Progress: {progress:.2%}, '
                f'Shoulder: {self.current_shoulder:.3f}, '
                f'Elbow: {self.current_elbow:.3f}'
            )

            time.sleep(step_period)
        
        self.current_shoulder = goal.target_shoulder
        self.current_elbow = goal.target_elbow

        self.publish_commanded_state(
            self.current_shoulder,
            self.current_elbow,
            0.0,
            0.0,
        )
        
        goal_handle.succeed()
        result = MoveToJointPosition.Result()
        result.success = True
        result.message = 'Goal completed successfully.'
        result.final_shoulder = goal.target_shoulder
        result.final_elbow = goal.target_elbow

        self.get_logger().info('Goal completed successfully.')
        return result
    
    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()
    
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()