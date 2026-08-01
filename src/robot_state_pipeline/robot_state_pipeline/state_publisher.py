import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.publish_state)

        self.elapsed_time = 0.0
        self.get_logger().info('State Publisher Node has been started. Publishing to /joint_states topic.')

    def publish_state(self) -> None:
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["shoulder_joint", "elbow_joint"]
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