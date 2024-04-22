import cv2
import os
import numpy as np

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

def create_video_from_images(image_folder, output_video_path, fps=30):
    # Get the list of image files in the folder
    image_files = [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]
    # image_files.sort()  # Sort the files to ensure proper order
    image_files = sorted(image_files, key=lambda x: int(os.path.splitext(os.path.basename(x))[0].split('_')[0]))

    if not image_files:
        print("No image files found in the folder.")
        return

    # Get the dimensions of the first image
    img = cv2.imread(image_files[0])
    height, width, _ = img.shape

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You may need to change the codec based on your system
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    cv2.waitKey(100)
    # Iterate through each image and write to video
    total_images = len(image_files)
    for i,image_file in enumerate(image_files):
        img = cv2.imread(image_file)
        print(f"Writing image {i+1}/{total_images} to video")
        out.write(img)

    # Release VideoWriter object
    out.release()
    print(f"Video created successfully at {output_video_path}")

def create_stacked_video(image_folders: list, output_video_path, horizontal_stack=True, fps=30):
    for image_folder in image_folders:
        assert os.path.exists(image_folder), f'Image folder {image_folder} does not exist'
    
    # Get the list of image files in the folder
    image_file_dict = {}
    for image_folder in image_folders:
        image_files = [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]
        image_files = sorted(image_files, key=lambda x: int(os.path.splitext(os.path.basename(x))[0].split('_')[0]))
        image_file_dict[image_folder] = image_files

    
    # Get the dimensions of the first image
    img = cv2.imread(image_file_dict[image_folders[0]][0])
    height, width, _ = img.shape

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You may need to change the codec based on your system
    if horizontal_stack:
        out = cv2.VideoWriter(output_video_path, fourcc, fps, (width * len(image_folders), height))
    else:
        out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height * len(image_folders)))

    cv2.waitKey(100)

    # Iterate through each image and write to video
    total_images = len(image_file_dict[image_folders[0]])
    for i in range(total_images):
        img_stacked = None
        for image_folder in image_folders:
            img = cv2.imread(image_file_dict[image_folder][i])
            if img_stacked is None:
                img_stacked = img
            elif horizontal_stack:
                img_stacked = np.hstack((img_stacked, img))
            else:
                img_stacked = np.vstack((img_stacked, img))

        print(f"Writing image {i+1}/{total_images} to video")
        out.write(img_stacked)    

    # Release VideoWriter object
    out.release()
    print(f"Video created successfully at {output_video_path}")


if __name__ == '__main__':
    # video_path = '/media/tyler/T7/warped_rgb/cam4_warped.mp4'
    # data_path = '/media/tyler/T7/warped_rgb/warped_cam4/'
    # fps = 0
    # extract_frames(video_path, data_path)

    # image_folder = "/media/tyler/Extreme SSD/zed2data/rgb_l/"
    # output_video_path = "/media/tyler/Extreme SSD/zed2data/rgb_l.mp4"
    # image_folder = "/home/tyler/Documents/SplaTAM/data/TartanAir/rgb/"
    # output_video_path = "/home/tyler/Documents/SplaTAM/data/TartanAir/rgb.mp4"
    # image_folder = "/home/tyler/Documents/SplaTAM/data/TartanAir/depth_est_colormap/"
    # output_video_path = "/home/tyler/Documents/SplaTAM/data/TartanAir/depth_est_colormap.mp4"
    # image_folder = "/home/tyler/Documents/SplaTAM/data/TartanAir/depth_colormap/"
    # output_video_path = "/home/tyler/Documents/SplaTAM/data/TartanAir/depth_colormap.mp4"
    # create_video_from_images(image_folder, output_video_path, fps=30)


    # image_folders = ["/home/tyler/Documents/SplaTAM/data/TartanAir/depth_est_colormap/", "/home/tyler/Documents/SplaTAM/data/TartanAir/depth_colormap/"]
    # output_video_path = "/home/tyler/Documents/SplaTAM/data/TartanAir/est_and_depth.mp4"
    image_folders = ["/media/tyler/Extreme SSD/zed2data/depth_colormap/", "/media/tyler/Extreme SSD/zed2data/rgb_L/"]
    output_video_path = "/media/tyler/Extreme SSD/zed2data/depth_and_rgb.mp4"
    create_stacked_video(image_folders, output_video_path, horizontal_stack=False, fps=30)



