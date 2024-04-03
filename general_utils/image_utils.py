    
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
    depth = np.clip(depth, 0, max_dist)
    depth = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
    depth = (depth * 255).astype(np.uint8)
    depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
    return depth

if __name__ == '__main__':
    print('Testing image_utils.py')
    img = read_image('test_rgb_image.png')
    cv2.imshow('test_rgb_image', img)
    cv2.waitKey(0) 
    depth = read_compressed_float('test_depth_image.png')
    depth = convert_dist_to_color(depth)
    cv2.imshow('test_depth', depth)
    cv2.waitKey(0) 
    print('Done testing image_utils.py')