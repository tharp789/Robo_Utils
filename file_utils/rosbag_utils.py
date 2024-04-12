import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import os

def extract_images_from_rosbag(rosbag_file, output_folder):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Initialize CvBridge
    bridge = CvBridge()

    # Open ROS bag
    with rosbag.Bag(rosbag_file, 'r') as bag:
        total_imgs = bag.get_message_count('/image')
        image_count = 0 
        with open('timestamps.txt', 'w') as f:
            for topic, msg, t in bag.read_messages():
                # Check if the message is an image
                if topic == '/image':  # Change '/camera_topic' to your actual image topic
                    try:
                        # Convert ROS image message to OpenCV image
                        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    except Exception as e:
                        print(e)
                        continue

                    # Get timestamp
                    timestamp = msg.header.stamp.to_nsec()

                    # Save image with timestamp as filename
                    image_filename = os.path.join(output_folder, f"{timestamp}.png")
                    cv2.imwrite(image_filename, cv_image)
                    print(f"Saved image {image_count}/{total_imgs} to {image_filename}")
                    image_count+=1

if __name__ == "__main__":
    rosbag_file = "/home/tyler/Documents/rosbag_mp4_wkspc/run4_cam0.bag"

    output_folder = "/media/tyler/Extreme SSD/Nardo_Runs/run4_night/run4_cam0_imgs/"

    extract_images_from_rosbag(rosbag_file, output_folder)