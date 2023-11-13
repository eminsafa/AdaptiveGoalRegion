import numpy as np

from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps
from adaptive_goal_region.robot_controller import RobotController
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler

def transform_position_with_orientation(original_orientation, orientation_quaternion):
    # Convert the quaternions to rotation objects
    original_rot = R.from_quat(original_orientation)
    relative_rot = R.from_quat(relative_orientation)

    # Apply the relative rotation to the original rotation
    converted_rot = relative_rot * original_rot

    # Return the converted orientation as a quaternion
    return converted_rot.as_quat()


pose_position = (-0.0038518, 0.690639, 0.962197)  # Replace with the actual position of the pose
orientation = (0.289911, 0.956686, -0.025359, 0.00782678)
relative_orientation = (0, 0, 0.38268, 0.92388)
new_ori = transform_position_with_orientation(orientation, relative_orientation)


robot_controller = RobotController(real_robot=False, group_id='manipulator')
robot_controller.go_to_pose_goal(robot_controller.create_pose(np.array(pose_position), np.array(new_ori)))


