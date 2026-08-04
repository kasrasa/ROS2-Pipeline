import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class StatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('state_publisher')
        robot_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(JointState, '/joint_states', robot_state_qos)
        self.robot_info_service = self.create_service(Trigger, '/robot_info', self.handle_robot_info_request)
        self.consumer = self.create_subscription(JointState, '/commanded_joint_states', self.handle_commanded_state, 10)

        self.robot_model = 'simulated_two_joint_inspection_robot'
        self.joint_names = [
            'shoulder_joint',
            'elbow_joint',
        ]
        self.current_positions = [0.0, 0.0]
        self.current_velocities = [0.0, 0.0]
        self.received_first_command = False
        
        self.declare_parameter('timer_frequency_hz', 10.0)
        publish_rate = self.get_parameter('timer_frequency_hz').get_parameter_value().double_value
        self.timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.5
        self.parameter_callback_handle = self.add_on_set_parameters_callback(self.validate_parameter_update)
        self.timer = self.create_timer(self.timer_period, self.publish_state)

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
    
    def handle_commanded_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            self.get_logger().error("Received commanded joint state message with inconsistent lengths.")
            return
        
        try:
            shoulder_index = msg.name.index('shoulder_joint')
            elbow_index = msg.name.index('elbow_joint')
        except ValueError as error:
            self.get_logger().error(
                f'Rejected commanded state: {error}'
            )
            return
        
        self.current_positions = [
            msg.position[shoulder_index],
            msg.position[elbow_index],
        ]

        self.current_velocities = [
            msg.velocity[shoulder_index],
            msg.velocity[elbow_index],
        ]
        
        self.received_first_command = True
        self.get_logger().info(
            f"Received commanded joint state: "
            f"shoulder: {self.current_positions[0]:.3f}, "
            f"elbow: {self.current_positions[1]:.3f}"
        )

    def publish_state(self) -> None:
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names.copy()
        joint_state_msg.position = self.current_positions.copy()
        joint_state_msg.velocity = self.current_velocities.copy()
        joint_state_msg.effort = []

        self.publisher.publish(joint_state_msg)

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