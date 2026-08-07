import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('camera_publisher')
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.publisher = self.create_publisher(Image, '/camera/image_raw', camera_qos)

        self.frame_count = 0
        self.declare_parameter('timer_frequency_hz', 30.0)
        publish_rate = self.get_parameter('timer_frequency_hz').get_parameter_value().double_value
        self.timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.5
        self.timer = self.create_timer(self.timer_period, self.publish_image)
        self.get_logger().info('Camera Publisher Node has been started. Publishing to /camera/image_raw topic.')

    def publish_image(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        msg.height = 480
        msg.width = 640
        
        msg.encoding = 'mono8'
        msg.step = msg.width
        msg.is_bigendian = 0
        msg.data = bytes([self.frame_count % 256] * (msg.height * msg.width))

        self.publisher.publish(msg)
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
