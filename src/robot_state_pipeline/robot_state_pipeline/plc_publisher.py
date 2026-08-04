import rclpy
from rclpy.node import Node
from robot_state_interfaces.msg import PlcState


class PLCStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('plc_state_publisher')
        self.publisher = self.create_publisher(PlcState, '/robot/plc_state', 10)

        self.state = 0

        self.declare_parameter('timer_frequency_hz', 10.0)
        publish_rate = self.get_parameter('timer_frequency_hz').get_parameter_value().double_value
        self.timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.5
        self.timer = self.create_timer(self.timer_period, self.publish_state)
        self.get_logger().info('PLC State Publisher Node has been started. Publishing to /plc/state topic.')

    def publish_state(self) -> None:
        msg = PlcState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        self.publisher.publish(msg)
        self.get_logger().info(f'Published PLC state: {self.state}')
        self.state = (self.state + 1) % 4

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PLCStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()