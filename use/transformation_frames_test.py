import time

import numpy as np

from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps
from adaptive_goal_region.robot_controller import RobotController


robot_controller = RobotController(real_robot=False, group_id='arm')
pose = np.array([-0.00694, 0.75659, 1.11397, -2.46658, 0.16737, -0.94575])
robot_controller.go_to_pose_goal(robot_controller.create_pose(pose[:3], pose[3:]))
robot_controller.create_object(robot_controller.create_pose(pose[:3], pose[3:]))
pose = robot_controller.frame_transformation(pose, "world", "panda_hand_tcp")
pose = robot_controller.pose_to_array(pose)
pose = robot_controller.frame_transformation(pose, "panda_hand_tcp", "panda_link8")
pose = robot_controller.pose_to_array(pose)
pose = robot_controller.frame_transformation(pose, "panda_link8", "world")
print(robot_controller.pose_to_array(pose))
robot_controller_2 = RobotController(real_robot=False, group_id='manipulator')
robot_controller_2.go_to_pose_goal(pose)
time.sleep(1)
ee_position = robot_controller.get_ee_position()
robot_controller.get_pose
print(ee_position)


