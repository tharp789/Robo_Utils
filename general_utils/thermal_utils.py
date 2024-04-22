import cv2
import numpy as np
import os


def enhance_image(image):
    # after hist_99, do clahe + bilateral filtering
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe = clahe.apply(image)
    bilateral = cv2.bilateralFilter(clahe, 5, 20, 15)
    return bilateral


def process_image(image_in, type):
    if type == "minmax":
        image_out = (image_in - np.min(image_in)) / (np.max(image_in) - np.min(image_in)) * 255
    elif type == "hist_99":
        if np.max(image_in) < 35000:
            image_out = (image_in - np.min(image_in)) / (np.max(image_in) - np.min(image_in)) * 255
        else:
            im_srt = np.sort(image_in.reshape(-1))
            upper_bound = im_srt[round(len(im_srt) * 0.99) - 1]
            lower_bound = im_srt[round(len(im_srt) * 0.01)]

            img = image_in
            img[img < lower_bound] = lower_bound
            img[img > upper_bound] = upper_bound
            image_out = ((img - lower_bound) / (upper_bound - lower_bound)) * 255.0
            image_out = image_out.astype(np.uint8)
    elif type == "hist_99_ours":
        if np.max(image_in) < 35000:
            image_out = (image_in - np.min(image_in)) / (np.max(image_in) - np.min(image_in)) * 255
        else:
            intensity, bin_edges = np.histogram(image_in, bins=4)
            upper_bound = bin_edges[1]
            lower_bound = bin_edges[0]

            image_out = np.zeros_like(image_in, dtype=np.uint8)
            mask = (image_in >= lower_bound) & (image_in <= upper_bound)
            image_out[mask] = ((image_in[mask] - lower_bound) / (upper_bound - lower_bound)).astype(np.uint8)
            image_out[image_in > upper_bound] = np.mean(image_out[image_in <= upper_bound])
            image_out = (image_out - np.min(image_out)) / (np.max(image_out) - np.min(image_out)) * 255
    else:
        image_out = image_in / 255

    return enhance_image(image_out.astype(np.uint8))

def read_and_enhance_thermals(data_path, output_data_path, type):
    if not os.path.exists(output_data_path):
        os.makedirs(output_data_path)
    file_list = os.listdir(data_path)
    file_list.sort()
    tot_img_num = len(file_list)
    for i,file in enumerate(file_list):
        if "png" in file:
            assert os.path.exists(data_path + file), "File not found: " + data_path + file
            img = cv2.imread(data_path + file, cv2.IMREAD_UNCHANGED)
            img = process_image(img, type)
            # cv2.imwrite(output_data_path + file, img)
            file = f'/{i}.png'
            cv2.imwrite(output_data_path + file, img)
            print("Processed: " + str(i) + " / " + str(tot_img_num) + " images")

if __name__ == '__main__':
    data_path = '/media/tyler/T7/thermal_inference/'
    output_data_path = '/media/tyler/T7/thermal_inference/enhanced/'
    # data_path = '/media/tyler/T7/field_dataset/frick_2_train/img_left/'
    # output_data_path = '/media/tyler/T7/field_dataset/frick_2_train/enhanced_left/'
    # data_path = '/media/tyler/T7/field_dataset/hawkins_3_train/img_left/'
    # output_data_path = '/media/tyler/T7/field_dataset/hawkins_3_train/enhanced_left/'
    # data_path = '/media/tyler/T7/field_dataset/hawkins_1_1_train/img_left/'
    # output_data_path = '/media/tyler/T7/field_dataset/hawkins_1_1_train/enhanced_left/'
    # data_path = '/media/tyler/T7/field_dataset/hawkins_4_train/img_left/'
    # output_data_path = '/media/tyler/T7/field_dataset/hawkins_4_train/enhanced_left/'
    read_and_enhance_thermals(data_path, output_data_path, "hist_99")
