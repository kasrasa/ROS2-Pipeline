import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('state_monitor')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.process_state,
            10
        )

        self.message_count = 0
        self.get_logger().info('State Monitor Node has been started. Subscribed to /joint_states topic.')

    
    def process_state(self, msg: JointState) -> None:
        self.message_count += 1

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