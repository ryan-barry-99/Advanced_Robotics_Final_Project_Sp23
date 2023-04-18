#!/usr/bin/env python3
'''
This script uses an OAK-D camera and a custom YOLOv8 model to track the cartesian coordinates and velocities of
a balloon in real time.
'''
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import math
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray


rospy.init_node('balloon_tracker')
pub = rospy.Publisher('balloon_velocity', Vector3, queue_size=10)
bbox_pub = rospy.Publisher('balloon_bbox', Float32MultiArray, queue_size=10)



# Get argument first
nnPath = "yolov8_weights2.blob"

if not Path(nnPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# Yolov8
labelMap = [
    "balloon"
]

syncNN = True
xLoc = [0]
yLoc = [0]
zLoc = [0]
timeSteps = [time.monotonic()]
vel_x = 0
vel_y = 0
vel_z = 0
# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
detectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
xoutRgb = pipeline.create(dai.node.XLinkOut)
nnOut = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)


xoutRgb.setStreamName("rgb")
nnOut.setStreamName("nn")
xoutDepth.setStreamName("depth")

# Properties
camRgb.setPreviewSize(416,416)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(40)

# Network specific settings
detectionNetwork.setConfidenceThreshold(0.85)
detectionNetwork.setNumClasses(1)
detectionNetwork.setCoordinateSize(4)


anchors = [0.10015182, 0.11443508, 0.10015182, 0.05721754, 0.05007591, 0.11443508, 0.23082139, 0.30423131, 0.23082139, 0.152115655, 0.115410695, 0.30423131, 0.44596353, 0.61528776, 0.44596353, 0.30764388, 0.222981765, 0.61528776]
detectionNetwork.setAnchors(anchors)
anchor_masks = {"side26": [1, 2, 3], "side13": [3, 4, 5]}
detectionNetwork.setAnchorMasks(anchor_masks)



detectionNetwork.setIouThreshold(0.5)
detectionNetwork.setBlobPath(nnPath)
detectionNetwork.setNumInferenceThreads(2)
detectionNetwork.input.setBlocking(False)


# Depth
monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)


# Spatial Detection network if OAK-D
# obj_det = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
detectionNetwork.setBoundingBoxScaleFactor(0.3)
detectionNetwork.setDepthLowerThreshold(100)
detectionNetwork.setDepthUpperThreshold(5000)
stereo.depth.link(detectionNetwork.inputDepth)

# Linking
camRgb.preview.link(detectionNetwork.input)
if syncNN:
    detectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

detectionNetwork.out.link(nnOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    qDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)


    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)


    def displayFrame(name, frame):
        global xLoc
        global yLoc
        global zLoc
        global timeSteps
        global vel_x
        global vel_y
        global vel_z
        color = (255, 0, 0)
        frame_height, frame_width = frame.shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2

        # Draw a vertical line at the center of the image
        cv2.line(frame, (416 // 2, 0), (416 // 2, 416), (0, 0, 0), 2)

        # Draw a horizontal line at the center of the image
        cv2.line(frame, (0, 416 // 2), (416, 416 // 2), (0, 0, 0), 2)

        for detection in detections:
            bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            # Publish bounding box to ROS topic
            bbox_msg = Float32MultiArray(data=bbox.tolist())
            bbox_pub.publish(bbox_msg)
            if detection.label < 0 or detection.label >= len(labelMap):
                print(f"Invalid label value: {detection.label}")
            else:
                spatialLocations = [detection.spatialCoordinates]
                for spatialLocation in spatialLocations:
                    x, y, z = spatialLocation.x / 1000, spatialLocation.y / 1000, spatialLocation.z / 1000
                    xLoc.insert(0, x)
                    yLoc.insert(0, y)
                    zLoc.insert(0, z)
                    timeSteps.insert(0, time.monotonic())
                    if len(xLoc) > 10 and len(yLoc) > 10 and len(timeSteps) > 10:
                        xLoc.pop()
                        yLoc.pop()
                        timeSteps.pop()
                        # calculate velocity using finite difference
                        vx = [0] * len(timeSteps)
                        vy = [0] * len(timeSteps)
                        vz = [0] * len(timeSteps)
                        for i in range(1, len(timeSteps)):
                            time_interval = timeSteps[i] - timeSteps[i - 1]
                            if time_interval == 0:
                                continue  # skip this iteration
                            vel_x = (xLoc[i] - xLoc[i - 1]) / time_interval
                            vel_y = (yLoc[i] - yLoc[i - 1]) / time_interval
                            vel_z = (zLoc[i] - zLoc[i - 1]) / time_interval
                            vx.append(vel_x)
                            vy.append(vel_y)
                            vz.append(vel_z)

                        # apply weights to the velocity estimates
                        alpha = 0.8
                        weights = [1]
                        for i in range(1, len(timeSteps)):
                            weight = math.exp(-alpha * (timeSteps[i] - timeSteps[i - 1]))
                            weights.append(weight)
                            vx[i] = weight * vx[i]
                            vy[i] = weight * vy[i]
                            vz[i] = weight * vz[i]

                        # compute the weighted average velocity
                        weighted_avg_vx = sum(vx) / sum(weights)
                        weighted_avg_vy = sum(vy) / sum(weights)
                        weighted_avg_vz = sum(vz) / sum(weights)
                        vel_msg = Vector3()
                        vel_msg.x = weighted_avg_vx
                        vel_msg.y = weighted_avg_vy
                        vel_msg.z = weighted_avg_vz
                        pub.publish(vel_msg)



    while True:
        if syncNN:
            inRgb = qRgb.get()
            inDet = qDet.get()
            depth = qDepth.get()
        else:
            inRgb = qRgb.tryGet()
            inDet = qDet.tryGet()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            depthFrame = depth.getFrame()  # depthFrame values are in millimeters
            cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                        (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

        if inDet is not None:
            detections = inDet.detections
            counter += 1

        if frame is not None:
            displayFrame("rgb", frame)

        if cv2.waitKey(1) == ord('q'):
            break
