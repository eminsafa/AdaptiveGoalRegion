from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)

robot_controller.hand_open()

robot_controller.go_to_capture_location()

robot_controller.capture_image_and_save_info()
