import numpy as np

def csv_to_txt(csv_path, txt_path):
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    with open(txt_path, 'w') as f:
        for line in lines:
            f.write(line.replace(',', ' '))
    print(f"Converted {csv_path} to {txt_path}")

if __name__ == "__main__":
    csv_to_txt("/media/tyler/Extreme SSD/Gascola_Processed_Top_Cams_04042024/Gascola_RawData/Pose_easy_000/cam_0_poses.csv",
                "/media/tyler/Extreme SSD/Gascola_Processed_Top_Cams_04042024/Gascola_RawData/Pose_easy_000/cam_0_poses.txt")