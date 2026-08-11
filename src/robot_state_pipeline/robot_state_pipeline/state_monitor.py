import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.event_handler import SubscriptionEventCallbacks
from sensor_msgs.msg import JointState


class StateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('state_monitor')
        robot_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        self.subscription = self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.process_state,
            robot_state_qos,
            event_callbacks=SubscriptionEventCallbacks(
                incompatible_qos=self.handle_qos_compatibility
            )
        )

        self.message_count = 0
        self.declare_parameter('stale_msg_threshold', 1.0)
        self.stale_msg_threshold = self.get_parameter('stale_msg_threshold').get_parameter_value().double_value
        self.parameter_callback_handle = self.add_on_set_parameters_callback(self._validate_parameter_update)
        self.get_logger().info('State Monitor Node has been started. Subscribed to /joint_states topic.')

    def _validate_parameter_update(self, params) -> SetParametersResult:
        for param in params:
            if param.name == 'stale_msg_threshold':
                if param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="stale_msg_threshold must be a positive double.")
                self.stale_msg_threshold = float(param.value)
                self.get_logger().info(f"Updated stale_msg_threshold to {self.stale_msg_threshold:.3f} seconds.")        
        return SetParametersResult(successful=True)
    
    def handle_qos_compatibility(self, event) -> None:
        self.get_logger().error(
        'Incompatible QoS discovered on /joint_states. '
        f'Total incompatibilities: {event.total_count}; '
        f'last policy kind: {event.last_policy_kind}'
    )
    
    def process_state(self, msg: JointState) -> None:
        self.message_count += 1
        current_time = self.get_clock().now()
        msg_timestamp = Time.from_msg(msg.header.stamp)
        msg_age = (current_time - msg_timestamp).nanoseconds / 1e9  # Convert to seconds
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().error(
                f"Received JointState message with uninitialized timestamp. "
                f"Message age cannot be determined."
            )
            return
        
        if msg_age > self.stale_msg_threshold:
            self.get_logger().warning(
                f"Received JointState message is {msg_age:.3f} seconds old. "
                f"Message may be stale."
            )
        elif msg_age < -0.1:
            self.get_logger().warning(
                f"Received JointState message has a timestamp from the future. "
                f"Message age: {msg_age:.3f} seconds."
            )

        validate_length = (len(msg.name) == len(msg.position) == len(msg.velocity)) and (len(msg.effort) in [0, len(msg.name)])
        if not validate_length:
            self.get_logger().error(
                f"Received JointState message with inconsistent lengths: "
                f"name({len(msg.name)}), position({len(msg.position)}), "
                f"velocity({len(msg.velocity)}), effort({len(msg.effort)})"
            )
            return

        shoulder_index = msg.name.index("shoulder_joint") if "shoulder_joint" in msg.name else None
        elbow_index = msg.name.index("elbow_joint") if "elbow_joint" in msg.name else None
        if shoulder_index is None or elbow_index is None:
            self.get_logger().error("Received JointState message missing 'shoulder_joint' or 'elbow_joint'.")
            return
        
        shoulder_position = msg.position[shoulder_index]
        elbow_position = msg.position[elbow_index]

        shoulder_velocity = msg.velocity[shoulder_index]
        elbow_velocity = msg.velocity[elbow_index]

        self.get_logger().info(
            f"Received JointState message with age {msg_age:.3f} seconds. "
            f"Message {self.message_count}: "
            f"shoulder position: {shoulder_position:.3f}, "
            f"elbow position: {elbow_position:.3f}, "
            f"shoulder velocity: {shoulder_velocity:.3f}, "
            f"elbow velocity: {elbow_velocity:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()