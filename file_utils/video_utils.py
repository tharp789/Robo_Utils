import cv2
import os
import multiprocessing
import shutil
import sys


# Define the paths to the video files
video_dir = '/media/tharp/Extreme SSD/Wildfire_Data/Hawkins'
video_paths = [f'{video_dir}/camera_0.mp4', f'{video_dir}/camera_4.mp4', f'{video_dir}/camera_5.mp4']
data_path = '/media/tharp/Extreme SSD/Wildfire_Data/Hawkins/Images'
fps = 0

# Create directories for each camera if they don't exist
for i in range(3):
    if not os.path.exists(f'{data_path}/cam{i}'):
        os.makedirs(f'{data_path}/cam{i}')

# Process each video
for idx, video_path in enumerate(video_paths):
    assert os.path.exists(video_path), f'Video {video_path} does not exist'
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f'Error opening video {video_path}')
        continue
    if fps==0:
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        print("fps",fps)

    print("processing", video_path)
    frame_number = 0
    num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # while True:
    for i in range(2):
        ret, frame = cap.read()
        if not ret: break
        frame_name = os.path.join(f'{data_path}/cam{idx}', f'{frame_number}.png')
        cv2.imwrite(frame_name, frame)
        print(f'Wrote frame {frame_number}/{num_frames} for camera {idx}')
        frame_number += 1
    cap.release()

print('Frames extraction completed!')