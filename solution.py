from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
import logging

#Configuring logger
logging.basicConfig(level=logging.INFO)

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 2

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)


    
if challengeLevel <= 2:
    control.start_keyboard_input()
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall
            scan = lidar.checkScan()
            distance_threshold = 0.5
            min_dist, min_dist_angle = lidar.detect_obstacle_in_cone(scan, distance_threshold, center=0, offset_angle=10)
            # if min_dist < distance_threshold:
            if min_dist != -1:
                print("Too Close")
                # control.stop_keyboard_control()
                # continuosly check for distance until the robot is far enough away
                  
                while min_dist != -1:
                    print('Moving robot backwards')
                    control.set_cmd_vel(-0.5, 0, 0.5)
                    time.sleep(1)
                    scan = lidar.checkScan()
                    distance_threshold = 0.5
                    # check distance
                    min_dist, min_dist_angle = lidar.detect_obstacle_in_cone(scan, distance_threshold, center=0, offset_angle=10)
                control.set_cmd_vel(0, 0, 0)
                print("Safe distance reached. Returning manual control.")
                control.start_keyboard_input()
                control.start_keyboard_control()
            else:
                print("safe distance")

            time.sleep(1)

            # while not TooCloseToWall():

            # 1. listen for lidar scans under a certain value (indicates we're too close to a wall):
                    

                # 2. once too close -> use control functions to move robot in opposite direction of wall:


                

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Get image and use ML_predict_stop_sign
            frame = camera.rosImg_to_cv2()
            detected, _, _, _, _ = camera.ML_predict_stop_sign(frame)
            if detected:
                print("Stop sign detected! Stopping robot.")
                control.set_cmd_vel(0, 0, 0)  # Stop the robot
                time.sleep(3)  # Wait for 3 seconds
                print("Resuming manual control.")
                control.start_keyboard_input()
                control.start_keyboard_control()

    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # 1. Collision avoidance with LIDAR
            scan = lidar.checkScan()
            min_dist, min_dist_angle = lidar.detect_obstacle_in_cone(scan, 0.5,0, 10)
            if min_dist != -1:
                print("Obstacle detected! Backing up.")
                control.set_cmd_vel(-0.5, 0, 0.5)
                time.sleep(1)
                control.set_cmd_vel(0, 0, 0)
                continue

            # 2. Stop sign detection with camera
            frame = camera.rosImg_to_cv2()
            detected, _, _, _, _ = camera.ML_predict_stop_sign(frame)
            if detected:
                print("Stop sign detected! Stopping robot.")
                control.set_cmd_vel(0, 0, 0)
                time.sleep(3)
                continue

            # 3. Default: move forward
            control.set_cmd_vel(0.5, 0, 0)

    if challengeLevel == 4:
        # Initialize variables for autonomous navigation
        target_angle = 0  # Target angle for navigation
        angle_threshold = 5  # Acceptable angle difference in degrees
        
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            
            # 1. Collision avoidance with LIDAR
            scan = lidar.checkScan()
            min_dist, min_dist_angle = lidar.detect_obstacle_in_cone(scan, 0.5, 0, 10)
            if min_dist != -1:
                print("Obstacle detected! Adjusting course.")
                # Turn away from obstacle
                turn_direction = 1 if min_dist_angle > 0 else -1
                control.set_cmd_vel(0, turn_direction * 0.5, 0.5)
                time.sleep(1)
                continue

            # 2. Stop sign detection
            frame = camera.rosImg_to_cv2()
            detected, _, _, _, _ = camera.ML_predict_stop_sign(frame)
            if detected:
                print("Stop sign detected! Stopping robot.")
                control.set_cmd_vel(0, 0, 0)
                time.sleep(3)
                continue

            # 3. Navigation using IMU
            quaternion = imu.checkImu()
            _, _, yaw = imu.euler_from_quaternion(quaternion)
            current_angle = math.degrees(yaw)
            angle_diff = current_angle - target_angle
            
            # Normalize angle difference to [-180, 180]
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360

            # Adjust heading if needed
            if abs(angle_diff) > angle_threshold:
                print(f"Adjusting heading. Current: {current_angle}, Target: {target_angle}")
                turn_direction = -1 if angle_diff > 0 else 1
                control.set_cmd_vel(0.3, turn_direction * 0.3, 0.5)
            else:
                # Move forward if heading is correct
                control.set_cmd_vel(0.5, 0, 0)

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
