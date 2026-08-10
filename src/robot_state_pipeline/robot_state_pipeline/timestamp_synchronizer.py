from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState

from robot_state_interfaces.msg import PlcState


@dataclass
class TimedSample:
    timestamp_ns: int
    message: object

class TimestampSynchronizer(Node):
    def __init__(self) -> None:
        super().__init__('timestamp_synchronizer')
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
        self.dropped_samples_count = 0

        self.create_subscription(JointState, '/robot/joint_states', self.joint_state_callback, 10)
        self.create_subscription(PlcState, '/robot/plc_state', self.plc_state_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, camera_qos_profile)

    def camera_callback(self, msg: Image) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds

        nearest_robot = self.synchronize(timestamp_ns, self.robot_buffer)
        nearest_plc = self.synchronize(timestamp_ns, self.plc_buffer)

        if nearest_robot is None:
            self.get_logger().warning(f"No robot joint state available for camera timestamp {timestamp_ns}.")
            return
        
        if nearest_plc is None:
            self.get_logger().warning(f"No PLC state available for camera timestamp {timestamp_ns}.")
            return

        robot_time_diff = abs(nearest_robot.timestamp_ns - timestamp_ns) / 1e9
        plc_time_diff = abs(nearest_plc.timestamp_ns - timestamp_ns) / 1e9

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
        
        self.get_logger().info(
            f"Synchronized sample for camera timestamp {timestamp_ns}: "
            f"Robot timestamp {nearest_robot.timestamp_ns}, PLC timestamp {nearest_plc.timestamp_ns}."
            f" Robot diff: {robot_time_diff:.3f}s, PLC diff: {plc_time_diff:.3f}s."
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
