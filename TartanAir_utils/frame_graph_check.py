
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

sys.path.insert(0, '/home/tyler/Documents/Robo_Utils/general_utils')
import transform_utils as transforms

def check_frame_graph(frame_graph_path):
    with open(frame_graph_path, 'r') as frame_graph_path:
        frame_graph = json.load(frame_graph_path)
    # frame_graph = json.load(frame_graph_path)
    transforms = frame_graph['transforms']
    camera_transforms = {}
    for transform in transforms:
        if transform['f0'] == 'rbf' and transform['f1'] != 'rpf':
            camera_transforms[transform['f1']] = transform

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # ax = plt.axes()
    for camera in camera_transforms:
        transform = camera_transforms[camera]
        position = np.array(transform['pose']['position'])
        rotation = transform['pose']['orientation']
        quat = [rotation['data']['x'], rotation['data']['y'], rotation['data']['z'], rotation['data']['w']]
        # ax.plot(position[0], position[1], 'ro')
        # ax.text(position[0], position[1], camera)
        ax.scatter(position[0], position[1], position[2], marker='o')
        ax.text(position[0], position[1], position[2], camera)
        lines = get_lines_for_coord_frame(quat, position, scaling=0.1)
        ax.plot([position[0], lines[0][0]], [position[1], lines[0][1]], [position[2], lines[0][2]], color='r')
        ax.plot([position[0], lines[1][0]], [position[1], lines[1][1]], [position[2], lines[1][2]], color='g')
        ax.plot([position[0], lines[2][0]], [position[1], lines[2][1]], [position[2], lines[2][2]], color='b')

    ax.set_title('Camera positions')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
    
def get_lines_for_coord_frame(quaternion, position, scaling=1.0):
    initial_points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    initial_points = scaling * initial_points
    initial_points = initial_points.T
    rotation_matrix = transforms.quat2rot(quaternion)
    rotated_points = np.dot(rotation_matrix, initial_points)
    rotated_points = rotated_points.T
    rotated_points += position
    lines = [rotated_points[1], rotated_points[2], rotated_points[3]]
    return lines

def visualize_trajectory(pose_path,max_frames=15):
    poses = load_poses(pose_path)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    for i in range(len(poses)):
        if i < max_frames:
            pose = poses[i]
            position = np.array(pose['position']) * -1
            orientation = pose['orientation']
            quat = [orientation[0], orientation[1], orientation[2], orientation[3]]
            ax.scatter(position[0], position[1], position[2], marker='o')
            ax.text(position[0], position[1], position[2], 'frame_' + str(i))
            lines = get_lines_for_coord_frame(quat, position, scaling=1)
            ax.plot([position[0], lines[0][0]], [position[1], lines[0][1]], [position[2], lines[0][2]], color='r')
            ax.plot([position[0], lines[1][0]], [position[1], lines[1][1]], [position[2], lines[1][2]], color='g')
            ax.plot([position[0], lines[2][0]], [position[1], lines[2][1]], [position[2], lines[2][2]], color='b')
        else:
            break
    ax.set_title('Camera positions')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def load_poses(pose_path):
    poses = []
    with open(pose_path, 'r') as pose_file:
        for line in pose_file:
            pose = line.strip().split(' ')
            pose_dict = {}
            pose_dict['position'] = [float(pose[0]), float(pose[1]), float(pose[2])]
            pose_dict['orientation'] = [float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6])]
            poses.append(pose_dict)
    return poses



if __name__ == '__main__':
    frame_graph_path = 'configs/frame_graph_artificial.json'
    # check_frame_graph(frame_graph_path)
    pose_path = '/home/tyler/Documents/Robo_Utils/data/Pose_000.txt'
    visualize_trajectory(pose_path)