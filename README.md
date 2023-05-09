# Balloon Trajectory Matching Omnidirectional Robot using YOLOv8

This project is designed to track a balloon using a custom YOLO model and a 3-wheeled omnidirectional drive robot, which can move in any direction without having to turn. The goal is to punch the balloon back into the air using the robot.

# Dataset and Model

The YOLOv8 model was trained on the V2 Balloon Detection dataset (https://www.kaggle.com/datasets/vbookshelf/v2-balloon-detection-dataset), resulting in efficient real-time detections at high accuracies. The model was trained using PyTorch, and the best weights were saved to the included yolov8_weights.pt file.

# Hardware Compatibility

For compatibility with DepthAI and OpenCV AI Kit hardware, the model must be saved in the Luxonis MyriadX Blob format. To achieve this, the weights were converted to the OpenVINO format using http://tools.luxonis.com/, and then the OpenVINO files were converted to the blob format using http://blobconverter.luxonis.com/.

# Code

The code for the ROS architecture that supports this robot is located in the Deployment folder. Files contained inside are as follows:
- QuadEncoder.py: Defines a class for extracting position and velocity data from quadrature encoders
- balloon_velocity.py: Deploys custom YOLOv8 model for detection and tracking, and publishes the velocity of a detected balloon
- forward_kinematics.py: Defines a class to compute the forward kinematics of an N-wheeled omnidirectional robot
- inverse_kinematics.py: Defines a class to compute the inverse kinematics of an N-wheeled omnidirectional robot
- motor.py: Defines a class for controlling a DC motor via software PWM signal
- pid_controller.py: Defines a PIDController class for use in updating wheel velocities
- pinout.py: Defines a dictionary containing the pinout for the robot. Updating pins here reflects the change globally
- robot_constants.py: Defines a dictionary containing the constants of an individual mobile robot. Updating the constants here reflects the change globally
- servo.py: Defines a class that enables PWM control for a servo
- update_target_velocity.py: Updates and publishes the robot's target velocity based on its current velocity and the balloon's velocity
- wheel_velocity_controller.py: Uses the PIDController class to match the robot's targeted and actual velocity

# Deployment Dependencies

- time
- Jetson.GPIO
- math
- pathlib
- sys
- opencv-python
- numpy
- rospy
