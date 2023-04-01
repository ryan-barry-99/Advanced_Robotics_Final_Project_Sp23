# Balloon Trajectory Matching Omnidirectional Robot using YOLO

This project aims to track a balloon using a custom YOLO model and punch it back into the air using a 3 wheeled omnidirectional drive robot. The robot moves in any direction without having to turn, which makes it perfect for this task. YOLOv8 was trained on the V2 Balloon dataset, allowing for efficient real time detections at high accuracies. 

The yolov8_balloon_detection jupyter notebook was used to train the model with PyTorch, with the resulting best weights saved to the included yolov8_weights.pt.

For compatibility with DepthAI and OpenCV AI Kit hardware, the model must be saved in the Luxonis MyriadX Blob format.
The weights were converted to the OpenVINO format with http://tools.luxonis.com/ with the OpenVINO files converted to the blob format with http://blobconverter.luxonis.com/
