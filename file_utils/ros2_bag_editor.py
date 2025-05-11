import rclpy
from rclpy.node import Node
import os
import shutil
import rclpy.serialization
from rosbag2_py import SequentialReader, SequentialWriter, TopicMetadata
from rosbag2_py._storage import StorageFilter, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message


class BagEditor(Node):
    def __init__(self, bag_path, output_bag, start_clip=0, duration=0, topic_exlude=[]):
        super().__init__('bag_image_extractor')
        self.bag_path = bag_path
        self.output_bag = output_bag
        self.topic_exclude = topic_exlude
        self.start_clip = start_clip
        self.duration = duration

        if os.path.exists(self.output_bag):
            shutil.rmtree(self.output_bag)

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

        self.writer = SequentialWriter()
        converter_options = ConverterOptions('', '')
        self.writer.open(StorageOptions(uri=self.output_bag, storage_id="mcap"), converter_options)
        for topic in topic_types:
            if topic.name not in self.topic_exclude:
                self.writer.create_topic(TopicMetadata(
                    name=topic.name,
                    type=topic.type,
                    serialization_format=topic.serialization_format
                ))
        self.start_time = None

    def edit_bag(self):
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            if topic in self.topic_exclude:
                continue
            if self.start_time is None:
                self.start_time = timestamp
            else:
                if timestamp - self.start_time < self.start_clip:
                    continue
                if self.duration > 0 and timestamp - (self.start_clip + self.start_clip) > self.duration:
                    break
            self.writer.write(topic, data, timestamp)

def main():
    rclpy.init()
    bag_path = "/media/tyler/hummingbird/wire_data_03-21-25"
    output_folder = "/media/tyler/hummingbird/wire_data_03_edit/"
    start_clip = 15
    duration = 138
    try:
        extractor = BagEditor(bag_path, output_folder, start_clip, duration)
        extractor.edit_bag()
    except Exception as e:
        print(f"An error occurred: {e}")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
        return  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()