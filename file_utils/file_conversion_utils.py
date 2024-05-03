import numpy as np
import os
import shutil
import cv2

def csv_to_txt(csv_path, txt_path):
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    with open(txt_path, 'w') as f:
        for line in lines:
            f.write(line.replace(',', ' '))
    print(f"Converted {csv_path} to {txt_path}")

def numpy_to_txt(numpy_path, txt_path):
    np_array = np.load(numpy_path)
    with open(txt_path, 'w') as f:
        for row in np_array:
            f.write(' '.join([str(x) for x in row]) + '\n')
    print(f"Converted {numpy_path} to {txt_path}")

def numpy_to_png(numpy_path, png_path, depth_scale=0.1):
    np_array = np.load(numpy_path)
    np_array = np_array / depth_scale
    cv2.imwrite(png_path, np_array)
    # hello = cv2.imread(png_path)
    # print(f"Converted {numpy_path} to {png_path}")

def convert_folder(input_folder, output_folder, conversion_function):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    file_list = os.listdir(input_folder)
    file_list.sort()
    total_files = len(file_list)
    for i, file in enumerate(file_list):
        if "npy" in file:
            input_path = os.path.join(input_folder, file)
            output_path = os.path.join(output_folder, file.replace(".npy", ".png"))
            conversion_function(input_path, output_path)
            print(f"Processed: {i+1}/{total_files} images")
    print(f"Finished processing all images from {input_folder} to {output_folder}")


def split_folder_by_file(input_folder, file_names, output_folder_names):
    assert len(file_names) == len(output_folder_names), "Number of file names and output folders must match."
    top_folder = os.path.dirname(input_folder)
    output_folders = [os.path.join(top_folder, output_folder_name) for output_folder_name in output_folder_names]
    for output_folder in output_folders:
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

    file_list = os.listdir(input_folder)
    file_list.sort()

    for file in file_list:
        for i, file_name in enumerate(file_names):
            if file_name in file:
                output_folder = output_folders[i]
                if not os.path.exists(output_folder):
                    os.makedirs(output_folder)
                os.rename(os.path.join(input_folder, file), os.path.join(output_folder, file))
                print(f"Moved {file} to {output_folder}")
                break
    print(f"Finished moving files from {input_folder} to {output_folders}")

def takeout_file_type(input_folder, file_type, output_folder, copy=False):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    file_list = os.listdir(input_folder)
    file_list.sort()
    total_files = len(file_list)
    for i,file in enumerate(file_list):
        if file_type in file:
            if copy:
                shutil.copy(os.path.join(input_folder, file), os.path.join(output_folder, file))
            else:
                os.rename(os.path.join(input_folder, file), os.path.join(output_folder, file))
            print(f"Moved {file} to {output_folder} - {i+1}/{total_files}")
    print(f"Finished moving files from {input_folder} to {output_folder}")

if __name__ == "__main__":
    # csv_to_txt("/media/tyler/Extreme SSD/Gascola_Processed_Top_Cams_04042024/Gascola_RawData/Pose_easy_000/cam_0_poses.csv",
    #             "/media/tyler/Extreme SSD/Gascola_Processed_Top_Cams_04042024/Gascola_RawData/Pose_easy_000/cam_0_poses.txt")

    # input_folder = '/media/tyler/Extreme SSD/zed2data/all_data/'
    # file_type = '_d'
    # output_folder = '/media/tyler/Extreme SSD/zed2data/depth_raw/'
    # takeout_file_type(input_folder, file_type, output_folder, copy=True)


    # input_folder = '/media/tyler/Extreme SSD/zed2data/all_poses.npy'
    # output_folder = '/media/tyler/Extreme SSD/zed2data/cam_poses.txt'
    # numpy_to_txt(input_folder, output_folder)
    # hmmm = cv2.imread('/home/tyler/Documents/SplaTAM/data/Replica/office0/results/depth000000.png')
    input_folder = '/home/tyler/Documents/SplaTAM/data/RealWorld/depth_processed/'
    output_folder = '/home/tyler/Documents/SplaTAM/data/RealWorld/depth/'
    convert_folder(input_folder, output_folder, numpy_to_png)




