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
                    scan = lidar.checkScan()
                    distance_threshold = 0.1
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
        # Load YOLO model (make sure the model can detect stop signs)
        model = YOLO('yolov8n.pt')  # Replace with your model path if needed

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Capture image from camera
            frame = camera.getImage()
            # Run YOLO detection
            results = model(frame)
            detected = False
            for result in results:
                for cls in result.boxes.cls:
                    # Class 11 is 'stop sign' in COCO dataset; check your model's class index
                    if int(cls) == 11:
                        detected = True
                        break
            if detected:
                print("Stop sign detected! Stopping robot.")
                control.set_cmd_vel(0, 0, 0)  # Stop the robot
                time.sleep(3)  # Wait for 3 seconds
                print("Resuming manual control.")
                control.start_keyboard_input()
                control.start_keyboard_control()

    if challengeLevel == 3:
        # Load YOLO model for stop sign detection
        model = YOLO('yolov8n.pt')  # Replace with your model path if needed
        distance_threshold = 0.5
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # 1. Collision avoidance with LIDAR
            scan = lidar.checkScan()
            min_dist, min_dist_angle = lidar.detect_obstacle_in_cone(scan, distance_threshold, center=0, offset_angle=10)
            if min_dist != -1:
                print("Obstacle detected! Backing up.")
                control.set_cmd_vel(-0.5, 0, 0.5)
                time.sleep(1)
                control.set_cmd_vel(0, 0, 0)
                continue
            # 2. Stop sign detection with camera and YOLO
            frame = camera.checkImage()
            results = model(frame)
            detected = False
            for result in results:
                for cls in result.boxes.cls:
                    if int(cls) == 11:  # COCO stop sign class
                        detected = True
                        break
            if detected:
                print("Stop sign detected! Stopping robot.")
                control.set_cmd_vel(0, 0, 0)
                time.sleep(3)
                continue
            # 3. Default: move forward
            control.set_cmd_vel(0.5, 0, 0)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

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
