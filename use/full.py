import time

from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps
from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)

robot_controller.hand_open()
#
robot_controller.go_to_capture_location()
#
time.sleep(5)

robot_controller.capture_image_and_save_info()

# # robot_controller.view_image()


# input(f"{robot_controller.latest_capture_path}\nWaiting for GPD, Press enter when ready!")
#
# poses = robot_controller.convert_grasping_poses()
#
visualize_grasps(None, poses)
