import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from mcap.reader import make_reader
from rosidl_runtime_py.utilities import get_message
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from sensor_msgs.msg import Image


class McapImageExtractor(Node):
    def __init__(self, bag_path, output_folder, topic_name='/rgb_image'):
        super().__init__('mcap_image_extractor')
        self.bag_path = bag_path
        self.output_folder = output_folder
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.image_count = 0

        # Ensure output folder exists
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder, exist_ok=True)

    def extract_images(self):
        with open(self.bag_path, "rb") as f:
            reader = make_reader(f)
            schema_dict = {}
            topic_type_map = {}

            for schema in reader.schemas:
                schema_dict[schema.id] = schema

            for channel in reader.channels:
                schema = schema_dict[channel.schema_id]
                topic_type_map[channel.topic] = schema.name

            for msg in reader.iter_messages(topics=[self.topic_name]):
                msg_type_str = topic_type_map.get(msg.channel.topic)
                if not msg_type_str:
                    continue
                msg_type = get_message(msg_type_str)
                self.process_image(msg.data, msg.log_time, msg_type)

        self.get_logger().info(f"Saved {self.image_count} images to {self.output_folder}")

    def process_image(self, raw_data, timestamp, msg_type):
        try:
            msg = deserialize_message(raw_data, msg_type)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if 'depth' in self.topic_name:
                cv_image = self.create_depth_viz(cv_image)
            timestamp_ns = str(timestamp)
            if cv_image.dtype == np.float32:
                filename = os.path.join(self.output_folder, f"{timestamp_ns}.npy")
                np.save(filename, cv_image)
            else:
                filename = os.path.join(self.output_folder, f"{timestamp_ns}.png")
                cv2.imwrite(filename, cv_image)
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def create_depth_viz(self, depth):
        min_depth = 0.5
        max_depth = 20.0
        depth[np.isnan(depth)] = min_depth
        depth[np.isinf(depth)] = min_depth
        depth[np.isneginf(depth)] = min_depth

        depth = (depth - min_depth) / (max_depth - min_depth)
        depth = (depth * 255).astype(np.uint8)
        depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
        return depth


def main():
    rclpy.init()
    bag_path = "/media/tyler/Storage/field_tests/street_wire_experiments/wire_tracking_05-07_40fov/wire_tracking_0.mcap"
    output_folder = "/media/tyler/Storage/field_tests/street_wire_experiments/wire_tracking_05-07_40fov/rgb"
    topic_name = '/wire_cam/zed_node/rgb/image_rect_color'
    try:
        extractor = McapImageExtractor(bag_path, output_folder, topic_name=topic_name)
        extractor.extract_images()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
