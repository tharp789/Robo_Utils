import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os

_CURRENT_PATH       = os.path.dirname(os.path.realpath(__file__))
_TOP_PATH           = os.path.join(_CURRENT_PATH, '..')
sys.path.append(_TOP_PATH)

from general_utils.transform_utils import make_transform_q, rot2quat, quat2rot 
from general_utils.image_utils import load_image, load_depth

class TartanAirDataLoader:
    def __init__(self, data_dir, trajectory_name, include_panoes=False):
        self.data_dir = data_dir
        self.run_name = data_dir.split('/')[-2]
        dir_list = os.listdir(data_dir)

        # get config files
        for path in dir_list:
            if "metadata.json" in path:
                metadata_path = data_dir + path
                with open(metadata_path, 'r') as metadata_file:
                    self.metadata = json.load(metadata_file)
            if "frame_graph.json" in path:
                frame_graph_path = data_dir + path
                with open(frame_graph_path, 'r') as frame_graph_file:
                    self.frame_graph = json.load(frame_graph_file)
                    self.strip_frame_graph()
            if "manifest.json" in path:
                manifest_path = data_dir + path
                with open(manifest_path, 'r') as manifest_file:
                    self.manifest = json.load(manifest_file)
            if os.path.isdir(data_dir + path):
                self.data_folder = data_dir + path
        trajectory_list = os.listdir(self.data_folder)
        for path in trajectory_list:
            if trajectory_name in path:
                self.trajectory_path = self.data_folder + '/' + path
        assert self.trajectory_path is not None, "Trajectory name given was not found"

        self.camera_list = []
        assert self.trajectory_path is not None, "Trajectory name given was not found"
        for path in os.listdir(self.trajectory_path):
            if "Pose" in path and ".txt" in path:
                self.pose_path = self.trajectory_path + '/' + path
            elif "cam" in path or "Cam" in path or "CAM" in path or 'rig' in path or 'Rig' in path or 'RIG' in path:
                if os.path.isdir(self.trajectory_path + '/' + path):
                    self.camera_list.append(path)
        if include_panoes:
            self.camera_list.append('rig_pano')

        self.strip_manifest()

        assert self.pose_path is not None, "Pose file not found"
        assert len(self.camera_list) > 0, "No camera folders found"
        
        # load pose file
        self.load_poses(include_panoes=include_panoes)
        # self.save_poses('')

        # load image file paths into a dictionaries
        self.depth_img_dict = {}
        self.rgb_img_dict = {}
        self.mask_dict = {}
        for cam in self.camera_list:
            if 'cam' in cam:
                camera_img_folder = self.trajectory_path + '/' + cam
                img_list = os.listdir(camera_img_folder)
                img_list.sort()
                self.depth_img_dict[cam] = {}
                self.rgb_img_dict[cam] = {}
                self.mask_dict[cam] = {}
                for img in img_list:
                    if "mask" in img:
                        self.mask_dict[cam] = load_image(camera_img_folder + '/' + img)
                    else:
                        img_index = int(img.split('.')[0].split('_')[0])
                        if "depth" in img or "Depth" in img or "Dist" in img or "dist" in img:
                            self.depth_img_dict[cam][img_index] = camera_img_folder + '/' + img
                        else:
                            self.rgb_img_dict[cam][img_index] = camera_img_folder + '/' + img
            elif 'rig' in cam and 'pano' not in cam:
                camera_img_folder = self.trajectory_path + '/' + cam
                img_list = os.listdir(camera_img_folder)
                img_list.sort()
                self.depth_img_dict[cam] = {}
                self.rgb_img_dict[cam] = {}
                self.mask_dict[cam] = {}
                if include_panoes:
                    self.depth_img_dict['rig_pano'] = {}
                    self.rgb_img_dict['rig_pano'] = {}
                    self.mask_dict['rig_pano'] = {}

                for img in img_list:
                    if "mask" in img:
                        self.mask_dict[cam] = load_image(camera_img_folder + '/' + img)
                    else:
                        img_index = int(img.split('.')[0].split('_')[0])
                        if "depth" in img or "Depth" in img or "Dist" in img or "dist" in img:
                            if 'Scene' in img and include_panoes:
                                self.depth_img_dict['rig_pano'][img_index] = camera_img_folder + '/' + img
                            else:
                                self.depth_img_dict[cam][img_index] = camera_img_folder + '/' + img
                        else:
                            if 'Scene' in img and include_panoes:
                                self.rgb_img_dict['rig_pano'][img_index] = camera_img_folder + '/' + img
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

    def load_poses(self, include_panoes):
        self.pose_dict = {}
        frame_num = 0
        ned2cv = np.array([[0,1,0,0],
                    [0,0,1,0],
                    [1,0,0,0],
                    [0,0,0,1]], dtype=np.float32)
        cv2ned = np.linalg.inv(ned2cv)
        
        with open(self.pose_path, "r") as f:
            lines = f.readlines()
            for cam in self.camera_list:
                if 'cam' in cam:
                    frame_num = 0
                    self.pose_dict[cam] = {}
                    for i, line in enumerate(lines):
                        r2w_ned = np.array(list(map(float, line.split())))
                        r2w_ned = make_transform_q(r2w_ned[3:], r2w_ned[:3])

                        r2c = self.camera_transforms_from_rig[cam]
                        r2c = make_transform_q(r2c[3:], r2c[:3])
                        c2w_ned = self.transform_pose(r2w_ned[:3,3], r2c[:3,3], r2w_ned[:3,:3])

                        c2im = self.image_transforms_from_camera[cam]

                        im2w_cv = ned2cv @ c2w_ned @ cv2ned @ c2im

                        self.pose_dict[cam][frame_num] = im2w_cv
                        frame_num += 1
                if cam == 'rig':
                    frame_num = 0
                    self.pose_dict[cam] = {}
                    if include_panoes:
                        self.pose_dict['rig_pano'] = {}
                    for i, line in enumerate(lines):
                        r2w_ned = np.array(list(map(float, line.split())))
                        r2w_ned = make_transform_q(r2w_ned[3:], r2w_ned[:3])
                        c2im = self.image_transforms_from_camera[cam]
                        im2w_cv = ned2cv @ r2w_ned @ cv2ned @ c2im
                        self.pose_dict[cam][frame_num] = im2w_cv

                        if include_panoes:
                            c2im_pano = self.image_transforms_from_camera['rig_pano']
                            im2w_cv_pano = ned2cv @ r2w_ned @ cv2ned @ c2im_pano
                            self.pose_dict['rig_pano'][frame_num] = im2w_cv_pano
                        frame_num += 1

    def save_poses(self, save_path, camera):
        poses = self.pose_dict[camera]
        with open(save_path, 'w') as f:
            for i in range(len(poses)):
                pose = poses[i]
                quat = rot2quat(pose[:3,:3])
                t = pose[:3,3]
                f.write(f"{t[0]} {t[1]} {t[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n")


    def strip_manifest(self, include_panoes=False):
        self.image_transforms_from_camera = {}
        # for camera in self.camera_list:
        #     self.image_transforms_from_camera[camera] = {}
        for sampler in self.manifest['samplers']:
            camera = sampler['mvs_cam_key']
            if 'cam' in camera:
                transform = np.eye(4)
                rotation_matrix = sampler['sampler']['orientation']['data']
                rotation_matrix = np.array(rotation_matrix).reshape(3,3)
                transform[:3,:3] = rotation_matrix
                self.image_transforms_from_camera[camera] = transform
            if 'rig' in camera:
                transform = np.eye(4)
                rotation_matrix = sampler['sampler']['orientation']['data']
                rotation_matrix = np.array(rotation_matrix).reshape(3,3)
                transform[:3,:3] = rotation_matrix
                if 'fisheye' or 'pinhole' in sampler['description']:
                    self.image_transforms_from_camera[camera]= transform
                else:
                    self.image_transforms_from_camera['rig_pano'] = transform
            
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

    def get_one_frame(self, frame_num):
        rgb_dict = {}
        depth_dict = {}
        pose_dict = {}
        for cam in self.camera_list:
            rgb_img = load_image(self.rgb_img_dict[cam][frame_num])
            depth_img = load_depth(self.depth_img_dict[cam][frame_num])
            c2w = self.pose_dict[cam][frame_num]
            rgb_dict[cam] = rgb_img
            depth_dict[cam] = depth_img
            pose_dict[cam] = c2w
        return rgb_dict, depth_dict, pose_dict
    

    def transform_pose(self, rig_pos, cam_pos, orientation):
            '''
            Transforms a local cam_pos w.r.t. to the rig frame into a point in the global NED AirSim Frame.
            '''
            #Rotate the cam_pos 3D vector by the orientation quaternion
            # rotcam = orientation * Quaternionr(*cam_pos, 0.0) * orientation.inverse()

            pos_delta = orientation @ cam_pos.reshape(3,1)

            #Shift the rotated local camera pose vector with the rig_pos to transform it to the NED AirSim global frame.
            newpos = rig_pos.reshape(3,1) + pos_delta
            newtransform = np.eye(4)
            newtransform[:3,3] = newpos.squeeze()
            newtransform[:3,:3] = orientation[:3,:3]
            return newtransform
    
def convert_pose_file(input_path, output_path, c2im):

    ned2cv = np.array([[0,1,0,0],
                [0,0,1,0],
                [1,0,0,0],
                [0,0,0,1]], dtype=np.float32)
    cv2ned = np.linalg.inv(ned2cv)

    temp = np.eye(4)
    temp[:3,:3] = c2im
    c2im = temp

    with open(input_path, "r") as in_file:
        with open(output_path, "w") as out_file:
            lines = in_file.readlines()
            for i, line in enumerate(lines):
                c2w_ned = np.array(list(map(float, line.split())))
                c2w_ned = make_transform_q(c2w_ned[3:], c2w_ned[:3])

                im2w_cv = ned2cv @ c2w_ned @ cv2ned @ c2im
                
                quat = rot2quat(im2w_cv[:3,:3])
                t = im2w_cv[:3,3]
                out_file.write(f"{t[0]} {t[1]} {t[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n")
    
    print("Pose file converted")
    

        
if __name__ == '__main__':
    # data_loader = TartanAirDataLoader('/media/tyler/Extreme SSD/Gascola_Processed_Top_Cams_04092024/','Pose_easy_000', include_panoes=True)
    # rgb_img, depth_img, pose = data_loader.get_one_frame(0)
    # print("RGB Image shape: ", rgb_img['cam0'].shape)

    input_path = "/home/tyler/Documents/SplaTAM/data/TartanAir/cam_0_poses_ned.txt"
    output_path = "/home/tyler/Documents/SplaTAM/data/TartanAir/cam_0_poses_cv.txt"
    c2im = np.array([[-1.0,  0.0,  0.0],
                     [0.0,  1.0,  0.0],
                     [0.0,  0.0,  -1.0]])
    
    convert_pose_file(input_path, output_path, c2im)

