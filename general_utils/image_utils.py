import cv2
import numpy as np
import os
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.pyplot as plt

def read_compressed_float(fn):
        img = read_image(fn, np.uint8)
        return np.squeeze( c4_uint8_as_float(img), axis=-1 )

def convert_folder_to_depth(input_folder, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    file_list = os.listdir(input_folder)
    file_list.sort()
    tot_img_num = len(file_list)
    for i, file in enumerate(file_list):
        if "png" in file:
            assert os.path.exists(input_folder + file), "File not found: " + input_folder + file
            img = read_compressed_float(input_folder + file)
            np.save(output_folder + file.replace(".png", ".npy"), img)
            print("Processed: " + str(i+1) + " / " + str(tot_img_num) + " images")
    
def c4_uint8_as_float(img):
    assert img.ndim == 3 and img.shape[2] == 4, f'img.shape = {img.shape}. Expecting image.shape[2] == 4. '
    
    # Check if the input array is contiguous.
    if ( not img.flags['C_CONTIGUOUS'] ):
        img = np.ascontiguousarray(img)

    return img.view('<f4')

def read_image(fn, dtype=np.uint8, enforce_3_c=False):
    img = cv2.imread(fn, cv2.IMREAD_UNCHANGED)
    assert img is not None, f'Failed to read {fn}. '
    img = img.astype(dtype)
    return enforce_3_channel(img) if enforce_3_c else img

def enforce_3_channel(img):
    '''
    Assuming img is an NumPy array.
    '''
    assert img.ndim in (2, 3), f'Image must be 2D or 3D. Got img.shape = {img.shape}. '
    return np.expand_dims(img, axis=-1) if img.ndim == 2 else img

def load_depth(file_path):
    depth = read_compressed_float(file_path)
    return depth

def load_image(file_path):
    image = read_image(file_path)
    return image

def convert_dist_to_color(depth, max_dist=10.0):
    # Normalize the depth for visualization
    depth[np.isnan(depth)] = 0
    if max_dist is not None:
        depth = np.clip(depth, 0, max_dist)
    depth = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
    depth = (depth * 255).astype(np.uint8)
    depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
    return depth

def convert_folder_to_depth_images(input_folder, output_folder, max_dist=30.0, compressed=False):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    file_list = os.listdir(input_folder)
    file_list.sort()
    tot_img_num = len(file_list)
    for i, file in enumerate(file_list):
        if "png" in file and compressed:
            assert os.path.exists(input_folder + file), "File not found: " + input_folder + file
            img = load_depth(input_folder + file)
            img = convert_dist_to_color(img, max_dist=max_dist)
            cv2.imwrite(output_folder + file, img)
            print("Processed: " + str(i+1) + " / " + str(tot_img_num) + " images")
        if "png" in file and not compressed:
            assert os.path.exists(input_folder + file), "File not found: " + input_folder + file
            img = cv2.imread(input_folder + file, cv2.IMREAD_UNCHANGED)
            img = convert_dist_to_color(img, max_dist=max_dist)
            cv2.imwrite(output_folder + file, img)
            print("Processed: " + str(i+1) + " / " + str(tot_img_num) + " images")
        elif 'npy' in file:
            assert os.path.exists(input_folder + file), "File not found: " + input_folder + file
            img = np.load(input_folder + file)
            img = convert_dist_to_color(img, max_dist=max_dist)
            cv2.imwrite(output_folder + file.replace(".npy", ".png"), img)
            print("Processed: " + str(i+1) + " / " + str(tot_img_num) + " images")
    print("Finished processing all images")

def load_as_float_depth(path):
    if "png" in path:
        depth = np.array(cv2.imread(path, cv2.IMREAD_UNCHANGED), dtype=np.float32)
    return depth

def depth2disp(depth):
    FOCAL_LENGTH = 406.33233091474426
    BASELINE = 0.24584925266278748
    min_depth = 1e-3
    mask = depth < min_depth
    disp = BASELINE * FOCAL_LENGTH / (depth + 1e-10)
    disp[mask] = 0.0
    return disp

def load_depth_as_disp(depth_path):
    if not os.path.exists(depth_path):
        return None

    depth = load_as_float_depth(depth_path) / 256.0
    disp = depth2disp(depth)
    return disp

def visualize_disp_as_numpy(disp, cmap="jet"):
    disp = np.nan_to_num(disp)  # change nan to 0

    vmin = np.percentile(disp[disp != 0], 0)
    vmax = np.percentile(disp[disp != 0], 95)

    normalizer = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
    mapper = cm.ScalarMappable(norm=normalizer, cmap=cmap)
    vis_data = (mapper.to_rgba(disp)[:, :, :3] * 255).astype(np.uint8)
    return vis_data

def convert_depth_folder_to_colormap(input_folder, output_folder, show_img=False):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    file_list = os.listdir(input_folder)
    file_list.sort()
    tot_img_num = len(file_list)
    for i, file in enumerate(file_list):
        if "png" in file:
            assert os.path.exists(input_folder + file), "File not found: " + input_folder + file
            img = load_depth_as_disp(input_folder + file)
            img = visualize_disp_as_numpy(img, cmap="jet")
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if show_img:
                cv2.imshow('depth', img)
                cv2.waitKey(0)  
            cv2.imwrite(output_folder + file, img)
            print("Processed: " + str(i+1) + " / " + str(tot_img_num) + " images")

if __name__ == '__main__':
    # print('Testing image_utils.py')
    # img = read_image('test_rgb_image.png')
    # cv2.imshow('test_rgb_image', img)
    # cv2.waitKey(0) 
    # depth = read_compressed_float('/media/tharp/Extreme SSD3/Gascola_Processed_Stereo_04042024/Gascola_RawData/Pose_easy_000/cam0/000003_pinholeDistance.png')
    # depth = convert_dist_to_color(depth, max_dist=30.0)
    # cv2.imwrite('pinhole_depth_image.png', depth)

    input = '/media/tyler/T7/wire_detection_bags/outside_test_4_extracted/depth/'
    output = '/media/tyler/T7/wire_detection_bags/outside_test_4_extracted/viz_depth/'
    convert_folder_to_depth_images(input, output, max_dist=30.0, compressed=False)