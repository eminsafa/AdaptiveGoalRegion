import math
import os
import time
from typing import (
    Any,
    List,
    Optional,
    Tuple,
    Union,
)

import moveit_commander
import numpy as np
import requests
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
)
from PIL import Image as PILImage
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import (
    CameraInfo,
    Image,
)
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from adaptive_goal_region.helper import create_new_directory
from adaptive_goal_region.object_helper import delete_model_from_gazebo, spawn_line_in_gazebo
import cv2


class RobotController:

    def __init__(self, real_robot: bool = False, group_id: str = "arm"):
        self.real_robot = real_robot
        self.robot_name = "fr3" if real_robot else "panda"
        rospy.init_node("panda_controller", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander(self.robot_name + '_' + group_id)
        self.hand_group = moveit_commander.MoveGroupCommander(self.robot_name + "_hand")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.tf_listener = TransformListener()
        self.cv_bridge = CvBridge()

        current_directory = os.path.dirname(__file__)
        self.save_dir = os.path.abspath(os.path.join(current_directory, '..', 'storage', 'captures'))
        self.latest_capture_dir = self.save_dir
        self.ee_link = self.move_group.get_end_effector_link()
        self.box_name = "YumYum_D3_Liquid"

        self.rgb_array = None
        self.depth_array = None
        self.camera_info = None

        self.latest_capture_path = None
        self.latest_grasp_result_path = None
        self.graspnet_url = "http://localhost:5000/run?path={path}"

        self.capture_joint_degrees = [1.57, -1.57, 0, -1.57, 0, 0.98, 0.7854]
        self.neutral_joint_values = [0.0, 0.4, 0.0, -1.78, 0.0, 2.24, 0.77]
        self.up_joints = [0.0, 0.0, 0.0, -1.78, 0.0, 2.24, 0.77]
        self.relase_joint_values = [1.39, 0.4, 0.0, -1.78, 0.0, 2.24, 0.77]
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.move_group.set_planning_time(1.9)
        time.sleep(1)  # wait to fill buffer

    # GRIPPER OPERATIONS

    def hand_open(self) -> None:
        # values -> {'panda_finger_joint1': 0.035, 'panda_finger_joint2': 0.035}
        self.hand_group.set_joint_value_target(self.hand_group.get_named_target_values("open"))
        self.hand_group.go(wait=True)

    def hand_close(self) -> None:
        # values -> {'panda_finger_joint1': 0.0, 'panda_finger_joint2': 0.0}
        self.hand_group.set_joint_value_target(self.hand_group.get_named_target_values("close"))
        self.hand_group.go(wait=True)

    def hand_grasp(self) -> None:
        target_values = {f'{self.robot_name}_finger_joint1': 0.006, f'{self.robot_name}_finger_joint2': 0.006}
        self.hand_group.set_joint_value_target(target_values)
        self.hand_group.go(wait=True)

    # POSE OPERATIONS

    @staticmethod
    def create_pose(position: np.ndarray, orientation: Optional[np.ndarray] = None) -> Pose:
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        if orientation is not None:
            if orientation.size == 4:
                pose.orientation.x = orientation[0]
                pose.orientation.y = orientation[1]
                pose.orientation.z = orientation[2]
                pose.orientation.w = orientation[3]
            elif orientation.size == 3:
                quaternion = quaternion_from_euler(
                    np.double(orientation[0]),
                    np.double(orientation[1]),
                    np.double(orientation[2]),
                )
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
        return pose

    def create_random_pose(self) -> Pose:
        position = np.random.uniform(0.1, 0.3, size=(3,))
        orientation = Rotation.random().as_quat()
        return self.create_pose(position, orientation)

    def get_ee_position(self) -> np.ndarray:
        position = self.move_group.get_current_pose().pose.position
        return np.array([
            position.x,
            position.y,
            position.z,
        ]).astype(np.float32)

    def rotate_quaternion(self, quaternions: np.ndarray, axis: str = 'z', degrees: float = -90.0) -> np.ndarray:
        original_rotation = R.from_quat(quaternions)
        original_rotation = R.from_euler(axis, degrees, degrees=True) * original_rotation
        return original_rotation.as_quat()

    def rotate_all_poses(self, pose_array: np.ndarray) -> np.ndarray:
        new_poses = []
        for pose in pose_array:
            pos1 = pose[:3]
            ori1_euler = pose[3:6]
            ori1_qua = quaternion_from_euler(
                np.double(ori1_euler[0]),
                np.double(ori1_euler[1]),
                np.double(ori1_euler[2]),
            )
            ori1_new = self.rotate_quaternion(ori1_qua, 'z', -45)
            ori1_new = euler_from_quaternion(ori1_new)
            pos2 = pose[6:9]
            ori2_euler = pose[9:]
            ori2_qua = quaternion_from_euler(
                np.double(ori2_euler[0]),
                np.double(ori2_euler[1]),
                np.double(ori2_euler[2]),
            )
            ori2_new = self.rotate_quaternion(ori2_qua, 'z', -45)
            ori2_new = euler_from_quaternion(ori2_new)
            new_poses.append(np.concatenate([pos1, ori1_new, pos2, ori2_new]))
        return np.array(new_poses)


    # PLANNING OPERATIONS

    def get_pose_goal_plan_with_duration(self, pose: Pose, planner: str) -> Tuple[Tuple, int]:
        if planner.upper() == 'RRT':
            self.move_group.set_planner_id("RRTConnect")
        elif planner.upper() == 'PRM':
            self.move_group.set_planner_id("PRMstar")
        self.move_group.set_pose_target(pose)
        start_time = time.time()
        plan = self.move_group.plan()
        end_time = time.time()
        planning_time = round((end_time - start_time) * 1000)  # in ms
        return plan, planning_time

    # MOVEMENT OPERATIONS

    def execute_plan(self, plan: Tuple) -> None:
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_capture_location(self) -> None:
        self.move_group.go(self.capture_joint_degrees, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_pose_goal(self, pose: Pose) -> None:
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_home_position(self) -> None:
        self.move_group.go(self.neutral_joint_values, True)

    def go_to_release_position(self) -> None:
        self.move_group.go(self.up_joints, True)
        self.move_group.go(self.relase_joint_values, True)

    def get_joint_values(self) -> List:
        return self.move_group.get_current_jonit_values()

    # CAMERA OPERATIONS

    def rgb_callback(self, msg: Any) -> None:
        self.rgb_array = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        time.sleep(1)

    def depth_callback(self, msg: Any) -> None:
        depth_img = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")
        self.depth_array = np.array(depth_img, dtype=np.dtype("f8"))
        time.sleep(1)

    def camera_info_callback(self, msg: Any) -> None:
        cam_info = msg.K
        self.camera_info = np.array([
            [cam_info[0], 0.0, cam_info[2]],
            [0.0, cam_info[4], cam_info[5]],
            [0.0, 0.0, 0.0],
        ])
        time.sleep(1)

    def filter_color(self, color_img: np.ndarray, depth_data: np.ndarray) -> np.ndarray:
        lower_red = np.array([0, 70, 50])
        upper_red = np.array([10, 255, 255])
        lower_brown = np.array([10, 20, 20])
        upper_brown = np.array([52, 255, 255])
        hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        mask_brown = cv2.inRange(hsv_image, lower_brown, upper_brown)
        mask_combined = cv2.bitwise_or(mask_red, mask_brown)
        mask_non_red_brown = cv2.bitwise_not(mask_combined)
        non_red_brown_objects_hsv = cv2.bitwise_and(hsv_image, hsv_image, mask=mask_non_red_brown)
        filtered_depth_data = cv2.bitwise_and(depth_data, depth_data, mask=mask_non_red_brown)
        return filtered_depth_data

    def capture_image_and_save_info(self) -> str:
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.camera_info_callback)
        rospy.sleep(5)
        depth_array = self.filter_color(np.array(self.rgb_array), np.array(self.depth_array))
        data_dict = {
            "rgb": np.array(self.rgb_array),
            "depth": depth_array / 1000.0,
            "label": np.zeros((720, 1280), dtype=np.uint8),
            "K": self.camera_info,
        }
        unique_save_dir = create_new_directory(self.save_dir)
        np.save(unique_save_dir + '/data.npy', data_dict)
        np.save(unique_save_dir + "/rgb.npy", np.array(self.rgb_array))
        np.save(unique_save_dir + "/depth.npy", np.array(self.depth_array) / 1000.0)
        self.latest_capture_path = unique_save_dir + '/data.npy'
        self.latest_capture_dir = unique_save_dir
        print("Data saved on", self.latest_capture_path)
        return self.latest_capture_path

    def view_image(self, file_path: Optional[str] = None) -> None:
        if file_path is None:
            file_path = self.latest_capture_dir + '/rgb.npy'
        data = np.load(file_path)
        cv2.imshow("Non-Red and Non-Brown Objects", data)
        cv2.waitKey(0)

    # CONTACT GRASPNET INTEGRATION

    def request_graspnet_result(self, path: Optional[str] = None, remote_ip: Optional[str] = None) -> Optional[str]:
        if path is None:
            if self.latest_capture_path is None:
                return None
            path = self.latest_capture_path
        print(f"REQUESTED PATH: {path}")
        try:
            if remote_ip:
                files = {'file': ('data.npy', open(path, 'rb'))}
                response = requests.get(remote_ip, files=files, timeout=30)
            else:
                response = requests.get(self.graspnet_url.format(path=path), timeout=30)
        except Exception as e:
            print(f"Request failed, please make sure Contact Graspnet Server is running!\n{e}")
            return None
        if response.status_code != 200:
            print("Grasping Pose Detection process failed!")
            return None

        if remote_ip:
            temp_file_path = self.save_dir + "/predictions.npz"
            with open(temp_file_path, 'wb') as target_file:
                target_file.write(response.content)
            print(f"Results Saved: {temp_file_path}")
            return temp_file_path
        else:
            print(f"Response Text: {response.text}")
            self.latest_grasp_result_path = response.text
            return response.text

    def process_grasping_results(self, path: Optional[str] = None) -> Optional[np.ndarray]:
        if path is None:
            if self.latest_grasp_result_path is None:
                return None
            path = self.latest_grasp_result_path

        data = np.load(path, allow_pickle=True)
        argmax = data['scores'].item()[-1].argmax()
        pred_grasp = data['pred_grasps_cam'].item()[-1][argmax]
        orientation = np.array((
            math.atan2(pred_grasp[2][1], pred_grasp[2][2]),
            math.asin(-pred_grasp[2][0]),
            math.atan2(pred_grasp[1][0], pred_grasp[0][0]),
        ))
        position = np.array((
            pred_grasp[0][3],
            pred_grasp[1][3],
            pred_grasp[2][3],
        ))
        result = np.concatenate((
            position,
            orientation,
        ))
        return result

    # FRAME TRANSFORMATION

    def transform_camera_to_world(self, cv_pose: Union[np.ndarray, list]) -> PoseStamped:
        pose = self.frame_transformation(cv_pose, "camera_depth_optical_frame", "world")
        if pose.pose.position.z < 0.5:
            time.sleep(1)
            print(f"Transformation Recalled for {cv_pose[:3]}")
            # return self.transform_camera_to_world(cv_pose)
            return self.frame_transformation(cv_pose, "camera_depth_optical_frame", "world")
        return pose

    def frame_transformation(self, pose_array: np.ndarray, frame_from: str, frame_to: str) -> PoseStamped:
        base_pose = PoseStamped()
        if pose_array.size == 6:
            quaternion = quaternion_from_euler(
                np.double(pose_array[3]),
                np.double(pose_array[4]),
                np.double(pose_array[5]),
            )
        else:
            quaternion = pose_array[3:]
        base_pose.pose.position.x = pose_array[0]
        base_pose.pose.position.y = pose_array[1]
        base_pose.pose.position.z = pose_array[2]
        base_pose.pose.orientation.x = quaternion[0]
        base_pose.pose.orientation.y = quaternion[1]
        base_pose.pose.orientation.z = quaternion[2]
        base_pose.pose.orientation.w = quaternion[3]
        base_pose.header.frame_id = frame_from

        result = self.tf_listener.transformPose(frame_to, base_pose)
        return result

    @staticmethod
    def matrix_to_pose_quaternion(matrix):
        position = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()
        pose_quaternion = np.concatenate((position, quaternion))

        return pose_quaternion

    def transform_grasping_poses(self) -> np.ndarray:
        with np.load("storage/grasping_poses/data.npz", allow_pickle=True) as data:
            data = dict(data)
        poses = []
        time.sleep(10)
        for i in data["pred_grasps_cam"].item()[-1]:
            raw_pose = self.matrix_to_pose_quaternion(i)
            ps = self.transform_camera_to_world(raw_pose)
            pose = np.array([
                ps.pose.position.x,
                ps.pose.position.y,
                ps.pose.position.z,
                ps.pose.orientation.x,
                ps.pose.orientation.y,
                ps.pose.orientation.z,
                ps.pose.orientation.w,
            ])
            poses.append(pose)
        return np.array(poses)


    # OBJECT CONTROLLER

    def set_base_pose(self, body: str, position: np.ndarray, orientation: np.ndarray) -> None:
        rospy.wait_for_service('/gazebo/set_model_state')
        for i in range(100):  # To avoid latency bug
            state_msg = ModelState()
            state_msg.model_name = body
            state_msg.pose.position.x = position[0]
            state_msg.pose.position.y = position[1]
            state_msg.pose.position.z = position[2]
            state_msg.pose.orientation.x = orientation[0]
            state_msg.pose.orientation.y = orientation[1]
            state_msg.pose.orientation.z = orientation[2]
            state_msg.pose.orientation.w = orientation[3]
            self.set_model_state_proxy(state_msg)

    def set_target_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        return self.set_base_pose(self.box_name, position, orientation)

    @staticmethod
    def pose_to_array(pose: Pose) -> np.ndarray:
        return np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]).astype(np.float32)

    def add_collision_object(self) -> None:
        p = PoseStamped()
        p.header.frame_id = self.robot_name + "_link0"
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -0.01
        self.scene.add_box("table", p, (2.0, 2.0, 0.1))

    def create_object(self, pose: Pose):
        delete_model_from_gazebo("line")
        time.sleep(1)
        spawn_line_in_gazebo("line", pose, 0.50, 0.005)



