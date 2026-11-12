import os

import rosbag2_py
from rclpy.serialization import serialize_message


class ForceProfileRecorder:

    def __init__(self, directory: str, name: str, topic: str, message_type: str, storage_id: str) -> None:
        os.makedirs(directory, exist_ok=True)

        self.path = os.path.join(directory, name)
        self.topic = topic

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=self.path, storage_id=storage_id),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(id=0, name=topic, type=message_type, serialization_format="cdr")
        )

    def write(self, message, timestamp_ns: int) -> None:
        self.writer.write(self.topic, serialize_message(message), timestamp_ns)

    def close(self) -> None:
        self.writer.close()
