import numpy as np

from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps
from adaptive_goal_region.robot_controller import RobotController
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler


robot_controller = RobotController(real_robot=False, group_id="manipulator")

pose_array = np.array([-0.00694, 0.75659, 1.11397, -2.46658, 0.16737, -0.94575])


pose = robot_controller.create_pose(pose_array[:3], pose_array[3:])
robot_controller.create_object(pose)

robot_controller.go_to_pose_goal(pose)

pose = robot_controller.frame_transformation(pose_array, "panda_hand_tcp", "panda_hand")

pose = robot_controller.pose_to_array(pose)

pose = robot_controller.create_pose(pose)

robot_controller.go_to_pose_goal(pose)


