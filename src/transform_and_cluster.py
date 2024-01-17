import argparse
import os
import time

from adaptive_goal_region.robot_controller import RobotController
from adaptive_goal_region.src.agr_helper import save_agr_data_as_txt
from adaptive_goal_region.src.agr_main import agr_new


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='AGR - Gazebo Image Capture and Save Results')

    # Add the arguments
    parser.add_argument('-i', '--input_file', type=str, required=True, help='The input file')
    parser.add_argument('-o', '--output_dir', type=str, required=True, help='The output dir')

    args = parser.parse_args()

    robot_controller = RobotController(real_robot=False)

    robot_controller.hand_open()
    robot_controller.go_to_capture_location()

    matrices_data = robot_controller.read_grasping_poses_file(args.input_file)
    poses, labels = robot_controller.convert_matrices_to_array_new(matrices_data)
    # poses = robot_controller.array_frame_transformation(poses)

    save_agr_data_as_txt(poses, os.path.join(args.output_dir, 'agr_output_single.txt'))
    adaptive_goal_region_data = agr_new(poses, labels)  # Includes 90 degree rotation
    save_agr_data_as_txt(adaptive_goal_region_data, os.path.join(args.output_dir, 'agr_output_region.txt'))

    time.sleep(3)
    print("CAPTURE COMPLETED!")

