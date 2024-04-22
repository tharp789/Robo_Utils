    
import cv2
import numpy as np

def read_compressed_float(fn):
        img = read_image(fn, np.uint8)
        return np.squeeze( c4_uint8_as_float(img), axis=-1 )
    
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
    if max_dist is not None:
        depth = np.clip(depth, 0, max_dist)
    depth = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
    depth = (depth * 255).astype(np.uint8)
    depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
    return depth

if __name__ == '__main__':

    # depth = read_compressed_float('/media/tharp/Extreme SSD3/Gascola_Processed_Stereo_04042024/Gascola_RawData/Pose_easy_000/cam0/000003_pinholeDistance.png')
    # depth = convert_dist_to_color(depth, max_dist=30.0)
    # cv2.imwrite('pinhole_depth_image.png', depth)

    # depth = read_compressed_float('/media/tharp/Extreme SSD3/Gascola_Processed_Top_Cams_04042024/Gascola_RawData/Pose_easy_000/cam0/000003_FisheyeDistance.png')
    # depth = convert_dist_to_color(depth, max_dist=30.0)
    # cv2.imwrite('fisheye_depth_image.png', depth)

    depth = np.load('/home/tyler/Documents/SplaTAM/data/TartanAir/depth_estimation/000000_pinhole.npy')
    depth = convert_dist_to_color(depth, max_dist=30.0)
    cv2.imshow('test_depth', depth)
    cv2.waitKey(0)

    # cv2.imshow('test_depth', depth)
    # cv2.waitKey(0) 
    # print('Done testing image_utils.py')