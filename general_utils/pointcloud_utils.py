import numpy as np
import open3d as o3d
import os

def read_pcd_as_numpy(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    return np.asarray(pcd.points)

def read_pcd_as_open3d(pcd_path):
    return o3d.io.read_point_cloud(pcd_path)

def combine_pcd_files_from_folder(pcd_folder, start_idx, amount):
    assert os.path.exists(pcd_folder), f"Folder {pcd_folder} does not exist"
    assert '.pcd' not in pcd_folder, f"Folder {pcd_folder} should be a folder, not a pcd file"
    combined_pcd = o3d.geometry.PointCloud()
    pcd_files = os.listdir(pcd_folder)
    pcd_files.sort()
    selected_pcd_files = pcd_files[start_idx:start_idx+amount]
    for pcd_file in selected_pcd_files:
        pcd_file_path = os.path.join(pcd_folder, pcd_file)
        pcd = o3d.io.read_point_cloud(pcd_file_path)
        combined_pcd += pcd
    return combined_pcd

def visualize_pcd(point_cloud):
    o3d.visualization.draw_geometries([point_cloud])

def save_pcd(point_cloud, save_path):
    o3d.io.write_point_cloud(save_path, point_cloud)

def downsample_pcd(point_cloud, voxel_size):
    return point_cloud.voxel_down_sample(voxel_size)

def render_image_from_pc_and_camera(point_cloud, camera_extrinsics, camera_intrinsics, camera_width, camera_height, max_depth=None):
        print("Projecting with Open3D...")
        # Project points onto the image plane
        render = o3d.cuda.pybind.visualization.rendering.OffscreenRenderer(width=camera_width, height=camera_height)
        cam = o3d.camera.PinholeCameraParameters()
        cam.extrinsic = camera_extrinsics
        cam.intrinsic = o3d.camera.PinholeCameraIntrinsic(camera_width, camera_height, camera_intrinsics)

        # cameraLines = o3d.geometry.LineSet.create_camera_visualization(intrinsic=cam.intrinsic, extrinsic=cam.extrinsic)
        # o3d.visualization.draw_geometries([point_cloud, cameraLines])

        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultUnlit"
        material.point_size = 1.0
        render.scene.set_background([0.0, 0.0, 0.0, 0.0])

        render.scene.add_geometry("point_cloud", point_cloud, material)
        render.setup_camera(cam.intrinsic, cam.extrinsic)
        depth_img = np.asarray(render.render_to_depth_image(z_in_view_space=True))

        if max_depth is not None:
            depth_img[depth_img < 0] = 0
            depth_img[depth_img > max_depth] = 0

        return depth_img

if __name__ == "__main__":
    pcd_path = "/media/tyler/Extreme SSD/Field_Tests/Gascola/run3_static/pcd_files"
    point_cloud = combine_pcd_files_from_folder(pcd_path, 0, 50)
    visualize_pcd(point_cloud)

