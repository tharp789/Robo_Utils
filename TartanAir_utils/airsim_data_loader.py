import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
import sys
import os

sys.path.insert(0, '/home/tyler/Documents/Robo_Utils/general_utils')
import transform_utils as transforms
import image_utils as io


class TartanAirDataLoader:
    def __init__(self, data_dir, trajectory_name):
        self.data_dir = data_dir
        self.run_name = data_dir.split('/')[-2]
        dir_list = os.listdir(data_dir)

        # get config files
        for path in dir_list:
            if "metadata" in path:
                metadata_path = data_dir + path
                with open(metadata_path, 'r') as metadata_file:
                    self.metadata = json.load(metadata_file)
            if "frame_graph" in path:
                frame_graph_path = data_dir + path
                with open(frame_graph_path, 'r') as frame_graph_file:
                    self.frame_graph = json.load(frame_graph_file)
                    self.strip_frame_graph()
            if "manifest" in path:
                manifest_path = data_dir + path
                with open(manifest_path, 'r') as manifest_file:
                    self.manifest = json.load(manifest_file)
            if os.path.isdir(data_dir + path):
                self.data_folder = data_dir + path
        trajectory_list = os.listdir(self.data_folder)
        for path in trajectory_list:
            if trajectory_name in path:
                self.trajectory_path = self.data_folder + '/' + path

        self.camera_list = []
        assert self.trajectory_path is not None, "Trajectory name given was not found"
        for path in os.listdir(self.trajectory_path):
            if "pose" in path or "Pose" in path or "POSE" in path:
                self.pose_path = self.trajectory_path + '/' + path
            elif "cam" in path or "Cam" in path or "CAM" in path:
                self.camera_list.append(path)
        
        # load pose file
        self.pose_dict = {}
        frame_num = 0
        with open(self.pose_path, 'r') as pose_file:
            for line in pose_file:
                pose = line.strip().split(' ')
                pose = [float(x) for x in pose]
                self.pose_dict[frame_num] = pose
                frame_num += 1

        # load image file paths into a dictionaries
        self.depth_img_dict = {}
        self.rgb_img_dict = {}
        self.mask_dict = {}
        for cam in self.camera_list:
            camera_img_folder = self.trajectory_path + '/' + cam
            img_list = os.listdir(camera_img_folder)
            img_list.sort()
            self.depth_img_dict[cam] = {}
            self.rgb_img_dict[cam] = {}
            self.mask_dict[cam] = {}
            for img in img_list:
                if "mask" in img:
                    self.mask_dict[cam] = io.load_image(camera_img_folder + '/' + img)
                else:
                    img_index = int(img.split('.')[0].split('_')[0])
                    if "depth" in img or "Depth" in img or "Dist" in img or "dist" in img:
                        self.depth_img_dict[cam][img_index] = camera_img_folder + '/' + img
                    else:
                        self.rgb_img_dict[cam][img_index] = camera_img_folder + '/' + img
        
        # remove any camera transforms that are not needed
        extra_cameras = []
        for key in self.camera_transforms_from_rig:
            if key not in self.camera_list:
                extra_cameras.append(key)
        for camera in extra_cameras:
            del self.camera_transforms_from_rig[camera]

        print("Data loader finished")

    def strip_frame_graph(self):
        transforms = self.frame_graph['transforms']
        camera_transforms = {}
        for transform in transforms:
            if transform['f0'] == 'rbf' and transform['f1'] != 'rpf':
                camera_transforms[transform['f1']] = transform    
        self.camera_transforms_from_rig = {}
        for transform in camera_transforms:
            camera = 'cam' + str(transform[3:])
            camera_position = np.array(camera_transforms[transform]['pose']['position'])
            camera_rotation = camera_transforms[transform]['pose']['orientation']
            quat = np.array([camera_rotation['data']['x'], camera_rotation['data']['y'], camera_rotation['data']['z'], camera_rotation['data']['w']])
            t_rig_cam = np.hstack((camera_position, quat))
            self.camera_transforms_from_rig[camera] = t_rig_cam

        
if __name__ == '__main__':
    data_loader = TartanAirDataLoader('/media/tyler/Extreme SSD/Gascola_Processed_Stereo_04022024/','Pose_easy_000')
    print(data_loader.dir_list)