import os
import threading
from datetime import datetime
from functools import partial

import rosbag2_py
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def message_type_name(message_type) -> str:
    package = message_type.__module__.split(".")[0]
    return f"{package}/msg/{message_type.__name__}"


class SensorRecorder:

    def __init__(self, node: Node, directory: str, topics: list[tuple[str, type]], storage_id: str, gated_topics: set[str] | None = None, callback_group: CallbackGroup | None = None, queue_depth: int = 100) -> None:
        directory = os.path.expanduser(directory)
        os.makedirs(directory, exist_ok=True)

        self.node = node
        self.topics = topics
        self.storage_id = storage_id
        self.gated_topics = gated_topics or set()
        self.gate_open = False
        self.callback_group = callback_group
        self.path = os.path.join(directory, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_depth,
        )

        self.lock = threading.Lock()
        self.writer: rosbag2_py.SequentialWriter | None = None
        self.subscriptions = []

    def start(self) -> None:
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=self.path, storage_id=self.storage_id),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )

        for index, (topic, message_type) in enumerate(self.topics):
            writer.create_topic(
                rosbag2_py.TopicMetadata(id=index, name=topic, type=message_type_name(message_type), serialization_format="cdr")
            )

        with self.lock:
            self.writer = writer

        for topic, message_type in self.topics:
            self.subscriptions.append(
                self.node.create_subscription(message_type, topic, partial(self.write, topic), self.qos, callback_group=self.callback_group, raw=True)
            )

    def set_gate(self, is_open: bool) -> None:
        with self.lock:
            self.gate_open = is_open

    def write(self, topic: str, data: bytes) -> None:
        timestamp = self.node.get_clock().now().nanoseconds

        with self.lock:
            if self.writer is None:
                return

            if topic in self.gated_topics and not self.gate_open:
                return

            self.writer.write(topic, data, timestamp)

    def stop(self) -> None:
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)

        self.subscriptions.clear()

        with self.lock:
            writer = self.writer
            self.writer = None

        if writer is not None:
            writer.close()
