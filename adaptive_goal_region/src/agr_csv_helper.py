from typing import List

import numpy as np


def numpy_array_to_str(array: np.ndarray) -> str:
    array = [str(i) for i in array]
    return f"""['{"','".join(array)}']"""


def matrix_to_str(matrix: np.ndarray) -> str:
    matrix = [numpy_array_to_str(i) for i in matrix]
    return f"""[{",".join(matrix)}]"""


def generate_lines(poses: np.ndarray) -> List:
    ind = poses.shape[1] // 2
    lines = ["poses,scores,contact_pts\n"]
    for pose in poses:
        pose_1 = numpy_array_to_str(pose[:ind])
        pose_2 = numpy_array_to_str(pose[ind:])
        lines.append(f'"{pose_1}"\n')
        lines.append(f'"{pose_2}"\n')
    return lines


def save_agr_data_as_csv(adaptive_goal_region_data: np.ndarray, name):
    csv_file_path = (
        "/home/"
        + "furkan"
        + "/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/hybridplanner-goal-regions/hybridplanner_common_bringup/src/results/agr_output_"
        + name
        + ".csv"
    )

    with open(csv_file_path, mode="w", newline="") as file:
        for line in generate_lines(adaptive_goal_region_data):
            file.write(line)
    name = ""
