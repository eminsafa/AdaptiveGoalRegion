import numpy as np

from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)

robot_controller.go_to_capture_location()

new_pose = robot_controller.transform_camera_to_world(np.array([
    0.38295, 0.21787, 1.01739, -2.32882, -0.01609, 0.02724,
]))

print(new_pose)
