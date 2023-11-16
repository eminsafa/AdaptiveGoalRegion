import time

from adaptive_goal_region.robot_controller import RobotController
from adaptive_goal_region.src.agr_helper import save_agr_data_as_txt
from adaptive_goal_region.src.agr_main import agr
from adaptive_goal_region.src.agr_csv_helper import save_agr_data_as_csv
# from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps

robot_controller = RobotController(real_robot=False)

robot_controller.hand_open()

# robot_controller.capture_joint_degrees = [1.57, -1.57, 0.0, -1.57, 0.0, 1.13446401, 0.7854]  # @todo check
robot_controller.go_to_capture_location()

robot_controller.capture_image_and_save_info()

input(f"{robot_controller.latest_capture_path}\nWaiting for GPD, Press enter when ready!")

matrices_data = robot_controller.read_grasping_poses_file()  # orientation as quaternion
poses = robot_controller.convert_matrices_to_array(matrices_data)  # orientation as quaternion
poses = robot_controller.array_frame_transformation(poses)  # orientation as quaternion

adaptive_goal_region_data = agr(poses, rotate=True, return_euler=True)  # Includes -45 degree rotation
save_agr_data_as_txt(adaptive_goal_region_data)  # NOTE THAT RESULTS MIGHT BE IN QUATERNION FORMAT, CHECK THE LINE ABOVE
save_agr_data_as_csv(adaptive_goal_region_data)
# visualize_grasps(adaptive_goal_region_data)
