import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
import sys
import os

sys.path.insert(0, '/home/tyler/Documents/Robo_Utils/general_utils')
from transform_utils import rot2quat, quat2rot, make_transform_q
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
        self.load_poses()

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

        self.num_imgs = len(self.rgb_img_dict[self.camera_list[0]])
        # remove any camera transforms that are not needed
        extra_cameras = []
        for key in self.camera_transforms_from_rig:
            if key not in self.camera_list:
                extra_cameras.append(key)
        for camera in extra_cameras:
            del self.camera_transforms_from_rig[camera]

        print("Data loader finished")

    def load_poses(self):
        self.pose_dict = {}
        frame_num = 0
        T = np.array([[0,1,0,0],
                    [0,0,1,0],
                    [1,0,0,0],
                    [0,0,0,1]], dtype=np.float32)
        T_inv = np.linalg.inv(T)

        for cam in self.camera_list:
            frame_num = 0
            self.pose_dict[cam] = {}
            with open(self.pose_path, "r") as f:
                lines = f.readlines()
            for line in lines:
                pvec = np.array(list(map(float, line.split())))
                r2w = make_transform_q(pvec[3:], pvec[:3])
                r2c = self.camera_transforms_from_rig[cam]
                r2c = make_transform_q(r2c[3:], r2c[:3])
                c2w = r2w @ np.linalg.inv(r2c)

                c2w = T.dot(c2w).dot(T_inv)
                # c2w = np.linalg.inv(c2w)
                self.pose_dict[cam][frame_num] = c2w
                frame_num += 1

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

    def get_one_frame(self, frame_num, cam_name):
        rgb_img = io.load_image(self.rgb_img_dict[cam_name][frame_num])
        depth_img = io.read_compressed_float(self.depth_img_dict[cam_name][frame_num])
        c2w = self.pose_dict[cam_name][frame_num]

        return rgb_img, depth_img, c2w
    

        
if __name__ == '__main__':
    data_loader = TartanAirDataLoader('/media/tyler/Extreme SSD/Gascola_Processed_Stereo_04022024/','Pose_easy_000')
    rgb_img, depth_img, pose = data_loader.get_one_frame(0, 'cam0')
    print(pose)