from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState

from robot_state_interfaces.msg import PlcState


@dataclass
class TimedSample:
    timestamp_ns: int
    message: object

class TimestampSynchronizer(Node):
    def __init__(self) -> None:
        super().__init__('timestamp_synchronizer')

        self.robot_buffer = deque(maxlen=500)
        self.plc_buffer = deque(maxlen=100)
        self.camera_buffer = deque(maxlen=10)

        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(JointState, '/robot/joint_states', self.joint_state_callback, 10)
        self.create_subscription(PlcState, '/robot/plc_state', self.plc_state_callback, 10)

    def camera_callback(self, msg: Image) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.camera_buffer.append(TimedSample(timestamp_ns, msg))
        self.synchronize(timestamp_ns, self.camera_buffer)
    
    def joint_state_callback(self, msg: JointState) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.robot_buffer.append(TimedSample(timestamp_ns, msg))
        self.synchronize(timestamp_ns, self.robot_buffer)
    
    def plc_state_callback(self, msg: PlcState) -> None:
        timestamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.plc_buffer.append(TimedSample(timestamp_ns, msg))
        self.synchronize(timestamp_ns, self.plc_buffer)

    @staticmethod
    def synchronize(timestamp_ns: int, samples: deque) -> None:
        if not samples:
            return None
        
        return min(samples, key=lambda sample: abs(sample.timestamp_ns - timestamp_ns))
