import numpy as np

from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)

pose = robot_controller.create_pose(np.array([0.3564, 0.86, .96, 0.83, 0.009, -0.01, 0.553]))

robot_controller.go_to_pose_goal(pose)


