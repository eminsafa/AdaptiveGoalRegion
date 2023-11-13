import time

from adaptive_goal_region.robot_controller import RobotController
from adaptive_goal_region.src.agr_helper import save_agr_data
from adaptive_goal_region.src.agr_main import agr
from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps

robot_controller = RobotController(real_robot=False)

robot_controller.hand_open()
# #
robot_controller.go_to_capture_location()
# #
# time.sleep(2)
# #
# robot_controller.capture_image_and_save_info()

input(f"{robot_controller.latest_capture_path}\nWaiting for GPD, Press enter when ready!")

poses = robot_controller.transform_grasping_poses()

adaptive_goal_region_data = agr(poses)

# adaptive_goal_region_data = robot_controller.rotate_all_poses(adaptive_goal_region_data)

save_agr_data(adaptive_goal_region_data)

# visualize_grasps(adaptive_goal_region_data)
