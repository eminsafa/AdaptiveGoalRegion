import time

import numpy as np
from scipy.spatial.transform import Rotation as R
from adaptive_goal_region.robot_controller import RobotController
from tf.transformations import quaternion_from_euler


def transform_position_with_orientation(original_orientation, orientation_quaternion):
    original_rot = R.from_quat(original_orientation)
    relative_rot = R.from_quat(relative_orientation)
    converted_rot = relative_rot * original_rot
    return converted_rot.as_quat()


file_path = "storage/spline_finals/agr_output.txt"
file = open(file_path, "r+")
poses = []
for line in file.read().split("\n"):
    if not line:
        continue
    pose = []
    for i in line.split(" "):
        if i:
            pose.append(float(i))
    poses.append(np.array(pose[:6]))
    poses.append(np.array(pose[6:]))

relative_orientation = (0, 0, -0.38268, 0.92388)

robot_controller = RobotController(real_robot=False, group_id="manipulator")

for pose in poses:
    orientation = quaternion_from_euler(np.double(pose[3]), np.double(pose[4]), np.double(pose[5]),)
    new_ori = transform_position_with_orientation(orientation, relative_orientation)
    pose = robot_controller.create_pose(pose[:3], new_ori)
    robot_controller.go_to_pose_goal(pose)
    time.sleep(1)

