import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import open3d as o3d
import os
from sensor_msgs import point_cloud2
import numpy as np

def extract_images_from_rosbag(rosbag_file_path, output_location, topic_name='/image'):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_location):
        os.makedirs(output_location)

    # Initialize CvBridge
    bridge = CvBridge()

    # Open ROS bag
    with rosbag.Bag(rosbag_file_path, 'r') as bag:
        total_imgs = bag.get_message_count(topic_name)
        image_count = 0 
        for topic, msg, t in bag.read_messages():
            # Check if the message is an image
            if topic == topic_name:  # Change '/camera_topic' to your actual image topic
                try:
                    # Convert ROS image message to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except Exception as e:
                    print(e)
                    continue

                # Get timestamp
                timestamp = msg.header.stamp.to_nsec()

                # Save image with timestamp as filename
                image_filename = os.path.join(output_location, f"{timestamp}.png")
                cv2.imwrite(image_filename, cv_image)
                print(f"Saved image {image_count}/{total_imgs} to {image_filename}")
                image_count+=1

def extract_pointclouds_from_rosbag(rosbag_file_path, output_location, topic_name='/velodyne_points'):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_location):
        os.makedirs(output_location)

    # Open ROS bag
    with rosbag.Bag(rosbag_file_path, 'r') as bag:
        total_pointclouds = bag.get_message_count(topic_name)
        pointcloud_count = 0
        for topic, msg, t in bag.read_messages():
            # Check if the message is a pointcloud
            if topic == topic_name:  # Change '/velodyne_points' to your actual pointcloud topic
                # Get timestamp
                timestamp = msg.header.stamp.to_nsec()
                # Generate a file name based on the topic and timestamp
                filename = "{}.pcd".format(str(timestamp))
                file_path = os.path.join(output_location, filename)
                
                # Convert the PointCloud2 message to PCD format
                points_msg = point_cloud2.read_points(msg)
                pointcloud = o3d.geometry.PointCloud()
                np_points = np.array(list(points_msg))
                pointcloud.points = o3d.utility.Vector3dVector(np_points[:, :3])
                o3d.io.write_point_cloud(file_path, pointcloud)

                print(f"Saved pointcloud {pointcloud_count}/{total_pointclouds} to {file_path}")
                pointcloud_count+=1

if __name__ == "__main__":
    rosbag_file = "/media/tyler/T7/wire_detection_bags/outside_test_4"

    output_folder = "/media/tyler/T7/wire_detection_bags/outside_test_4/rgb_images"

    extract_pointclouds_from_rosbag(rosbag_file, output_folder, topic_name='/zed/zed_node/rgb/image_rect_color')