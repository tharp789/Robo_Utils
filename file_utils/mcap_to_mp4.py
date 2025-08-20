import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory as Ros2DecoderFactory
from rosidl_runtime_py.utilities import get_message
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from sensor_msgs.msg import Image

from sensor_msgs.msg import CameraInfo

class McapToMp4(Node):
    def __init__(self, bag_folder, rgb_topic_name, depth_topic_name, 
                 depth_camera_info_topic_name, rgb_camera_info_topic_name):
        super().__init__('mcap_image_extractor')
        self.bag_folder = bag_folder
        self.rgb_topic_name = rgb_topic_name
        self.depth_topic_name = depth_topic_name
        self.depth_camera_info_topic_name = depth_camera_info_topic_name
        self.rgb_camera_info_topic_name = rgb_camera_info_topic_name
        self.depth_camera_info = None
        self.rgb_camera_info = None
        self.bridge = CvBridge()
        self.image_count = 0
        self.camera_info_processed = {}

        if not os.path.isdir(self.bag_folder):
            self.bag_files = [self.bag_folder]
            self.bag_folder = os.path.dirname(self.bag_folder)
        else:
            self.bag_files = [os.path.join(self.bag_folder, f) for f in os.listdir(self.bag_folder) if f.endswith('.mcap')]

        self.rgb_output_folder = os.path.join(self.bag_folder, "rgb_images")
        self.depth_output_folder = os.path.join(self.bag_folder, "depth_images")
        self.depth_viz_output_folder = os.path.join(self.bag_folder, "depth_viz_images")

        # Ensure output folder exists
        if not os.path.exists(self.rgb_output_folder):
            os.makedirs(self.rgb_output_folder, exist_ok=True)
        if not os.path.exists(self.depth_output_folder):
            os.makedirs(self.depth_output_folder, exist_ok=True)
        if not os.path.exists(self.depth_viz_output_folder):
            os.makedirs(self.depth_viz_output_folder, exist_ok=True)