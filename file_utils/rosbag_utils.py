import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import os

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

                # Save pointcloud with timestamp as filename
                pointcloud_filename = os.path.join(output_location, f"{timestamp}.pcd")
                with open(pointcloud_filename, 'w') as f:
                    f.write(msg)
                print(f"Saved pointcloud {pointcloud_count}/{total_pointclouds} to {pointcloud_filename}")
                pointcloud_count+=1

if __name__ == "__main__":
    rosbag_file = "/home/tyler/Documents/rosbag_mp4_wkspc/run4_cam0.bag"

    output_folder = "/media/tyler/Extreme SSD/Nardo_Runs/run4_night/run4_cam0_imgs/"

    extract_images_from_rosbag(rosbag_file, output_folder)