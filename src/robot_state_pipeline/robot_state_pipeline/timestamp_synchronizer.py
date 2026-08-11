from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, JointState

from robot_state_interfaces.msg import PlcState, AlignmentReport


@dataclass
class TimedSample:
    timestamp_ns: int
    message: object

class TimestampSynchronizer(Node):
    def __init__(self) -> None:
        super().__init__('timestamp_synchronizer')
        # some metrics
        self.processed_images = 0
        self.valid_alignments = 0
        self.missing_robot_states = 0
        self.missing_plc_states = 0
        self.avg_robot_error = 0.0
        self.robot_error_sum = 0.0
        self.avg_plc_error = 0.0
        self.plc_error_sum = 0.0
        self.dropped_samples_count = 0
        self.valid_ratio = 0.0


        self.report_publihser_rate_hz = self.declare_parameter('report_publihser_rate_hz', 1.0)
        publish_rate = self.report_publihser_rate_hz.get_parameter_value().double_value
        self.timer_period = 1.0 / publish_rate if publish_rate > 0 else 1.0
        # timer for periodic metrics logging
        self.report_timer = self.create_timer(self.timer_period, self.report_metrics)
        self.report_publisher = self.create_publisher(AlignmentReport, '/alignment_report', 10)


        camera_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.robot_buffer = deque(maxlen=500)
        self.plc_buffer = deque(maxlen=100)
        self.camera_buffer = deque(maxlen=10)

        self.joint_state_tol = 0.01  # 10 ms tolerance for joint state synchronization
        self.plc_state_tol = 0.2  # 200 ms tolerance for PLC state synchronization

        self.create_subscription(JointState, '/robot/joint_states', self.joint_state_callback, 10)
        self.create_subscription(PlcState, '/robot/plc_state', self.plc_state_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, camera_qos_profile)


    def camera_callback(self, msg: Image) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.processed_images += 1

        nearest_robot = self.synchronize(timestamp_ns, self.robot_buffer)
        nearest_plc = self.synchronize(timestamp_ns, self.plc_buffer)

        if nearest_robot is None:
            self.missing_robot_states += 1
            self.get_logger().warning(f"No robot joint state available for camera timestamp {timestamp_ns}.")
            return
        
        if nearest_plc is None:
            self.missing_plc_states += 1
            self.get_logger().warning(f"No PLC state available for camera timestamp {timestamp_ns}.")
            return

        robot_time_diff = abs(nearest_robot.timestamp_ns - timestamp_ns) / 1e9
        self.robot_error_sum += robot_time_diff

        plc_time_diff = abs(nearest_plc.timestamp_ns - timestamp_ns) / 1e9
        self.plc_error_sum += plc_time_diff

        sample_valid = (
            robot_time_diff <= self.joint_state_tol and
            plc_time_diff <= self.plc_state_tol
        )
        
        if not sample_valid:
            self.dropped_samples_count += 1
            self.get_logger().warning(
                f"Dropping sample for camera timestamp {timestamp_ns} due to time difference. "
                f"Robot diff: {robot_time_diff:.3f}s, PLC diff: {plc_time_diff:.3f}s. "
                f"Total dropped samples: {self.dropped_samples_count}"
            )
            return
        
        self.valid_alignments += 1
        self.get_logger().info(
            f"Synchronized sample for camera timestamp {timestamp_ns}: "
            f"Robot timestamp {nearest_robot.timestamp_ns}, PLC timestamp {nearest_plc.timestamp_ns}."
            f" Robot diff: {robot_time_diff:.3f}s, PLC diff: {plc_time_diff:.3f}s."
        )
    
    def report_metrics(self) -> None:
        self.avg_robot_error = self.robot_error_sum / self.valid_alignments if self.valid_alignments > 0 else 0.0
        self.avg_plc_error = self.plc_error_sum / self.valid_alignments if self.valid_alignments > 0 else 0.0
        self.valid_ratio = (self.valid_alignments / self.processed_images) if self.processed_images > 0 else 0.0

        self.msg = AlignmentReport()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.processed_images = self.processed_images
        self.msg.valid_alignments = self.valid_alignments
        self.msg.dropped_samples_count = self.dropped_samples_count
        self.msg.missing_robot_states = self.missing_robot_states
        self.msg.missing_plc_states = self.missing_plc_states
        self.msg.avg_robot_error = self.avg_robot_error
        self.msg.avg_plc_error = self.avg_plc_error
        self.msg.valid_ratio = self.valid_ratio

        self.report_publisher.publish(self.msg)

        self.get_logger().info(
            f"Metrics Report: "
            f"Processed Images: {self.processed_images}, "
            f"Valid Alignments: {self.valid_alignments}, "
            f"Missing Robot States: {self.missing_robot_states}, "
            f"Missing PLC States: {self.missing_plc_states}, "
            f"Dropped Samples: {self.dropped_samples_count}, "
            f"Average Robot Error: {self.avg_robot_error:.6f}s, "
            f"Average PLC Error: {self.avg_plc_error:.6f}s, "
            f"Valid Ratio: {self.valid_ratio:.2%}"
        )
    
    def joint_state_callback(self, msg: JointState) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.robot_buffer.append(TimedSample(timestamp_ns, msg))
    
    def plc_state_callback(self, msg: PlcState) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.plc_buffer.append(TimedSample(timestamp_ns, msg))

    @staticmethod
    def synchronize(timestamp_ns: int, samples: deque) -> None:
        if not samples:
            return None
        
        return min(samples, key=lambda sample: abs(sample.timestamp_ns - timestamp_ns))
    
def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimestampSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
