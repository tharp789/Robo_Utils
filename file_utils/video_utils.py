import cv2
import os

def extract_frames(video_path, data_path, fps=0):
    if not os.path.exists(data_path):
        os.makedirs(data_path)

    # Process each video
    assert os.path.exists(video_path), f'Video {video_path} does not exist'
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f'Error opening video {video_path}')
        return
    
    if fps==0:
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        print("fps",fps)

    print("processing", video_path)
    frame_number = 0
    num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # while True:
    for i in range(num_frames):
        ret, frame = cap.read()
        if not ret: break
        frame_name = os.path.join(f'{data_path}/{frame_number}.png')
        cv2.imwrite(frame_name, frame)
        print(f'Wrote frame {frame_number}/{num_frames} to {frame_name}')
        frame_number += 1

    cap.release()
    print('Frames extraction completed!')


if __name__ == '__main__':
    video_path = '/media/tyler/T7/gascola_staticrun_cam0_equirectangular.mp4'
    data_path = '/media/tyler/T7/warped_smoke_frames'
    fps = 0
    extract_frames(video_path, data_path)

