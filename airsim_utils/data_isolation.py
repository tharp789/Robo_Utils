import numpy as np
import os
import cv2

def get_single_view_images(data_dir,camera_ints, direction):

    new_data_dir = data_dir + '/config_direction_check/'
    if not os.path.exists(new_data_dir):
        os.makedirs(new_data_dir)

    for camera_int in camera_ints:
        new_camera_dir = new_data_dir + '/cam' + str(camera_int) + '/'
        if not os.path.exists(new_camera_dir):
            os.makedirs(new_camera_dir)
        camera_dir = data_dir + '/cam' + str(camera_int) + '/'
        images = []
        image_list = os.listdir(camera_dir)
        image_list.sort()
        for image in image_list:
            if direction in image and 'Depth' not in image:
                img = cv2.imread(camera_dir + image)
                images.append(img)
        for i in range(len(images)):
            cv2.imwrite(new_camera_dir + 'cam' + str(camera_int) +'_img' + str(i) + '.png', images[i])
    



if __name__ == '__main__':
    images = get_single_view_images('/home/tyler/Documents/Robo_Utils/data/Gascola_Data/', [0, 4, 5], 'top')

