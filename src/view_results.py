import argparse
import time

import numpy as np

from visualizer import visualize_grasps


def read_floats_from_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            float_values = [float(value) for value in line.split()]
            data.append(float_values)
    return data


def get_data(path):
    with np.load(path, allow_pickle=True) as data:
        return dict(data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='AGR - Gazebo Image Capture and Save Results')

    # Add the arguments
    parser.add_argument('-i', '--input_dir', type=str, required=True, help='The input file')

    args = parser.parse_args()

    results = read_floats_from_file(args.input_dir+'/agr_output_region.txt')
    data = get_data(args.input_dir+'/poses.npz')


    visualize_grasps(
        full_pc=data['pc_full'],
        pred_grasps_cam=data['pred_grasps_cam'].item(),
        scores=data['scores'].item(),
        plot_opencv_cam=True,
        pc_colors=data['pc_colors'],
        adaptive_goal_region_data=results,
        single=False
    )


