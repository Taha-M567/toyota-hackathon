# Toyota Hackathon - TurtleBot Challenge

## Overview
This project is for the Toyota Hackathon robotics challenge, where a TurtleBot is programmed to navigate a course with increasing levels of autonomy and awareness. The robot uses keyboard control, LIDAR, and camera-based object detection (YOLO) to complete various tasks.

## Challenge Levels

### Level 0: Pure Keyboard Control
- Manually control the robot using the keyboard.

### Level 1: Keyboard Control with Safety Features
- Collision detection with walls using LIDAR.
- Robot stops and backs up before colliding.

### Level 2: Keyboard Control with Awareness
- Stop sign detection using YOLO and the camera.
- Robot stops for 3 seconds at stop signs, then resumes manual control.

### Level 3: Autonomous Control and Static Obstacles
- Robot moves forward autonomously.
- Avoids obstacles using LIDAR.
- Stops at stop signs using YOLO.

## How to Run
1. Clone this repository:
   ```sh
   git clone https://github.com/Taha-M567/toyota-hackathon.git
   cd toyota-hackathon
   ```
2. Install dependencies (see requirements below).
3. Set the `challengeLevel` variable in `solution.py` to the desired level (0-3).
4. Run the script:
   ```sh
   python solution.py
   ```

## Requirements
- Python 3.x
- ROS2
- numpy
- ultralytics (for YOLO)
- TMMC_Wrapper (provided by organizers)

## Notes
- For stop sign detection, ensure you have a YOLO model (e.g., `yolov8n.pt`) that can detect stop signs.
- The code is designed to work both in simulation and on the actual robot.

## Repository
[https://github.com/Taha-M567/toyota-hackathon](https://github.com/Taha-M567/toyota-hackathon)

---

If you have any questions or issues, please open an issue on the repository.
