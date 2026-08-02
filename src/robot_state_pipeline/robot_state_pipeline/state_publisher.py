import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class StatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.robot_info_service = self.create_service(Trigger, '/robot_info', self.handle_robot_info_request)

        self.robot_model = 'simulated_two_joint_inspection_robot'
        self.joint_names = [
            'shoulder_joint',
            'elbow_joint',
        ]
        
        self.declare_parameter('timer_frequency_hz', 2.0)
        publish_rate = self.get_parameter('timer_frequency_hz').get_parameter_value().double_value
        self.timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.5
        self.parameter_callback_handle = self.add_on_set_parameters_callback(self.validate_parameter_update)
        self.timer = self.create_timer(self.timer_period, self.publish_state)

        self.elapsed_time = 0.0
        self.get_logger().info('State Publisher Node has been started. Publishing to /joint_states topic.')

    def validate_parameter_update(self, params) -> SetParametersResult:
        for param in params:
            if param.name == 'timer_frequency_hz':
                if param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER) or param.value <= 0:
                    return SetParametersResult(successful=False, reason="timer frequency must be a positive double/integer.")
                self.timer_period = 1.0 / float(param.value)
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(self.timer_period, self.publish_state)
                self.get_logger().info(f"Updated timer_frequency_hz to {param.value:.3f} seconds. New publish rate: {1.0 / self.timer_period:.3f} Hz.")
        return SetParametersResult(successful=True)
    
    def handle_robot_info_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        current_freq_hz = 1.0 / self.timer_period

        response.success = True
        response.message = (
            f"Robot Model: {self.robot_model}\n"
            f"Joint Counts: {len(self.joint_names)}\n"
            f"Joint Names: {', '.join(self.joint_names)}\n"
            f"Current Publish Rate: {current_freq_hz:.3f} Hz"
        )

        self.get_logger().info("Robot info service was called. Responding with robot information.")

        return response

    def publish_state(self) -> None:
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names.copy()
        joint_state_msg.position = [
            math.sin(self.elapsed_time),
            math.cos(self.elapsed_time)
        ]
        joint_state_msg.velocity = [
            math.cos(self.elapsed_time),
            -math.sin(self.elapsed_time)
        ]
        joint_state_msg.effort = []

        self.publisher.publish(joint_state_msg)
        self.elapsed_time += self.timer_period

        self.get_logger().info(
            f"shoulder position: {joint_state_msg.position[0]:.3f}, "
            f"elbow position: {joint_state_msg.position[1]:.3f}"
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()