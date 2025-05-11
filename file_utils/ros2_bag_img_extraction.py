import rclpy
from rclpy.node import Node
import os
import cv2
import numpy as np
import rclpy.serialization
from rosbag2_py import SequentialReader
from rosbag2_py._storage import StorageFilter, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message


class BagImageExtractor(Node):
    def __init__(self, bag_path, output_folder, topic_name='/rgb_image'):
        super().__init__('bag_image_extractor')
        self.bag_path = bag_path
        self.output_folder = output_folder
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.image_count = 0
        self.reader = SequentialReader()
        storage_options = StorageOptions(
            uri=self.bag_path,
            storage_id='mcap')
        converter_options = ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        topic_types = self.reader.get_all_topics_and_types()
        self.type_map = {topic.name: topic.type for topic in topic_types}

        # Create output directory
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder, exist_ok=True)

    def extract_images(self):

        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            if topic == self.topic_name:
                msg_type_str = self.type_map[topic]
                msg_type = get_message(msg_type_str)
                self.process_image(data, timestamp, msg_type)

        self.get_logger().info(f"Saved {self.image_count} images to {self.output_folder}")

    def process_image(self, raw_data, timestamp, msg_type):
        try:
            # Deserialize the message
            msg = rclpy.serialization.deserialize_message(raw_data, msg_type)
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
    bag_path = "/mnt/Storage/field_tests/street_wire_experiments/wire_tracking_05-07_40fov/"
    output_folder = "/mnt/Storage/field_tests/street_wire_experiments/wire_tracking_05-07_40fov/rgb"
    topic_name = '/wire_cam/zed_node/rgb/image_rect_color'
    try:
        # extractor = BagImageExtractor(args.bag_path, args.output_folder)
        extractor = BagImageExtractor(bag_path, output_folder, topic_name=topic_name)
        extractor.extract_images()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()