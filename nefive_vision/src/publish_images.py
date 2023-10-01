#!/usr/bin/env python3

import cv2
import time
import yaml
import io
import signal  # for ctrl-C handling
import sys
from cv_bridge import CvBridge
import numpy as np
import depthai as dai
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

debugMode = False

def signal_handler(signal, frame):
    print('CTRL-C caught, exiting.')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)



temporalSettings = {
    "OFF":  dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.PERSISTENCY_OFF,
    "8in8": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_8_OUT_OF_8,
    "2in3": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_3,
    "2in4": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4,
    "2in8": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_OUT_OF_8,
    "1in2": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_2,
    "1in5": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_5,
    "1in8": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_8,
    "indef": dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.PERSISTENCY_INDEFINITELY
    }

medianMap = {
    "OFF": dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF,
    "3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
    "5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
    "7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
}

def parse_calibration_yaml(params):
    cam_info = CameraInfo()
    cam_info.height = params['image_height']
    cam_info.width = params['image_width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['camera_matrix']['data']
    cam_info.D = params['distortion_coefficients']['data']
    cam_info.R = params['rectification_matrix']['data']
    cam_info.P = params['projection_matrix']['data']

    return cam_info

fps = 12

# Create pipeline
print("Creating pipeline...")
pipeline = dai.Pipeline()

print("Creating device...")
device = dai.Device()
queueNames = []

# Define sources and outputs
leftCam = pipeline.create(dai.node.ColorCamera)
rightCam = pipeline.create(dai.node.ColorCamera)
stereo = pipeline.create(dai.node.StereoDepth)
# stereo.enableDistortionCorrection(True)

leftOut = pipeline.create(dai.node.XLinkOut)
rightOut = pipeline.create(dai.node.XLinkOut)
depthOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)

leftOut.setStreamName("left")
rightOut.setStreamName("right")
depthOut.setStreamName("depth")
disparityOut.setStreamName("disp")

queueNames.append("left")
queueNames.append("right")
queueNames.append("depth")
queueNames.append("disp")

scaleNumerator   = 1
scaleDenominator = 4
height = 1200 * (scaleNumerator / scaleDenominator)
width = 1920 * (scaleNumerator / scaleDenominator)

# The disparity is computed at this resolution, then upscaled to RGB resolution
colorResolution = dai.ColorCameraProperties.SensorResolution.THE_1200_P

leftCam.setResolution(colorResolution)
leftCam.setBoardSocket(dai.CameraBoardSocket.CAM_B)
leftCam.setIspScale(scaleNumerator, scaleDenominator) 
leftCam.setFps(fps)

rightCam.setResolution(colorResolution)
rightCam.setBoardSocket(dai.CameraBoardSocket.CAM_C)
rightCam.setIspScale(scaleNumerator, scaleDenominator)
rightCam.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereo.initialConfig.setConfidenceThreshold(245)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)

print("Aligning to right camera...")
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)

stereo.initialConfig.setMedianFilter(medianMap["7x7"])  # KERNEL_7x7 default
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
stereo.setExtendedDisparity(False)
stereo.setSubpixel(True)

print("Creating post processing config...")
config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = False
config.postProcessing.temporalFilter.persistencyMode = temporalSettings["OFF"]

config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 25
config.postProcessing.spatialFilter.numIterations = 1
config.postProcessing.thresholdFilter.minRange = 50
config.postProcessing.thresholdFilter.maxRange = 2000
config.postProcessing.decimationFilter.decimationFactor = 1
stereo.initialConfig.set(config)

print("Creating links...")
# Linking
leftCam.video.link(leftOut.input)
rightCam.video.link(rightOut.input)

leftCam.isp.link(stereo.left)
rightCam.isp.link(stereo.right)

stereo.depth.link(depthOut.input)
stereo.disparity.link(disparityOut.input)

left_img_pub = rospy.Publisher('stereo/left/image', Image, queue_size=1)
right_img_pub = rospy.Publisher('stereo/right/image', Image, queue_size=1)
depth_img_pub = rospy.Publisher('stereo/depth/image', Image, queue_size=1)
disparity_img_pub = rospy.Publisher('stereo/disparity/image', Image, queue_size=1)

left_cam_pub = rospy.Publisher('stereo/right/camera_info', CameraInfo, queue_size=1)
right_cam_pub = rospy.Publisher('stereo/left/camera_info', CameraInfo, queue_size=1)
depth_cam_pub = rospy.Publisher('stereo/depth/camera_info', CameraInfo, queue_size=1)
disparity_cam_pub = rospy.Publisher('stereo/disparity/camera_info', CameraInfo, queue_size=1)

left_compressed_pub = rospy.Publisher('stereo/left/image/compressed', CompressedImage, queue_size=1)
right_compressed_pub = rospy.Publisher('stereo/right/image/compressed', CompressedImage, queue_size=1)
disparity_compressed_pub = rospy.Publisher('stereo/disparity/image/compressed', CompressedImage, queue_size=1)

# init messages
left_img_msg = Image()
left_img_msg.height = height
left_img_msg.width = width
left_img_msg.step = width * 3  # bytes per row: pixels * channels * bytes per channel (1 normally)
left_img_msg.encoding = 'rgb8'
left_img_msg.header.frame_id = 'left_camera'  # TF frame

right_img_msg = Image()
right_img_msg.height = height
right_img_msg.width = width
right_img_msg.step = width * 3
right_img_msg.encoding = 'rgb8'
right_img_msg.header.frame_id = 'right_camera'

depth_img_msg = Image()
depth_img_msg.height = height
depth_img_msg.width = width
depth_img_msg.step = width * 3  # bytes per row: pixels * channels * bytes per channel (1 normally)
depth_img_msg.encoding = 'rgb8'
depth_img_msg.header.frame_id = 'right_camera'  # TF frame

disparity_img_msg = Image()
disparity_img_msg.height = height
disparity_img_msg.width = width
disparity_img_msg.step = width * 3
disparity_img_msg.encoding = 'rgb8'
disparity_img_msg.header.frame_id = 'right_camera'

left_img_compressed = CompressedImage()
right_img_compressed = CompressedImage()
disparity_img_compressed = CompressedImage()

left_img_compressed.format = "jpeg"
right_img_compressed.format = "jpeg"
disparity_img_compressed.format = "jpeg"


# parse the left and right camera calibration yaml files
left_cam_info = rospy.get_param("left_cam_info")
right_cam_info = depth_cam_info = disparity_cam_info = rospy.get_param("right_cam_info")
left_cam_info_msg = parse_calibration_yaml(left_cam_info)
right_cam_info_msg = parse_calibration_yaml(right_cam_info)

rospy.init_node('stereo_pub')
br = CvBridge()

# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)

    print("Pipeline started.")
    frameRight = None
    frameDisp = None
    frameLeft = None
    frameDepth = None

    if debugMode == True:
        # Configure windows; trackbar adjusts blending ratio of rgb/depth
        leftWindowName = "left"
        rightWindowName = "right"
        depthWindowName = "depth"
        dispWindowName = "disp"

        cv2.namedWindow(leftWindowName)
        cv2.namedWindow(rightWindowName)
        cv2.namedWindow(depthWindowName)
        cv2.namedWindow(dispWindowName)

    while True:
        latestPacket = {}
        latestPacket["left"] = None
        latestPacket["right"] = None
        latestPacket["depth"] = None
        latestPacket["disp"] = None

        queueEvents = device.getQueueEvents(("left", "right", "depth", "disp"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]
        
        stamp = rospy.Time.now()
            
        if latestPacket["left"] is not None:
            frameLeft = latestPacket["left"].getCvFrame()
            if debugMode == True:
                cv2.imshow(leftWindowName, frameLeft)

            left_img_msg.header.stamp = stamp
            left_cam_info_msg.header.stamp = stamp
            left_cam_pub.publish(left_cam_info_msg)
            left_img_pub.publish(br.cv2_to_imgmsg(frameLeft, encoding="bgr8"))

            left_img_compressed.header.stamp = stamp
            left_img_compressed.data = np.array(cv2.imencode('.jpg', frameLeft)[1]).tobytes()
            left_compressed_pub.publish(left_img_compressed)

        if latestPacket["right"] is not None:
            frameRight = latestPacket["right"].getCvFrame()
            if debugMode == True:
                cv2.imshow(rightWindowName, frameRight)

            right_img_msg.header.stamp = stamp
            right_cam_info_msg.header.stamp = stamp
            right_cam_pub.publish(right_cam_info_msg)
            right_img_pub.publish(br.cv2_to_imgmsg(frameRight, encoding="bgr8"))

            right_img_compressed.header.stamp = stamp
            right_img_compressed.data = np.array(cv2.imencode('.jpg', frameRight)[1]).tobytes()
            right_compressed_pub.publish(right_img_compressed)

        if latestPacket["depth"] is not None:
            frameDepth = latestPacket["depth"].getCvFrame()
            if debugMode == True:
                cv2.imshow(depthWindowName, frameDepth)

            # depth_img_msg.header.stamp = stamp
            # depth_cam_info.header.stamp = stamp
            # depth_cam_pub.publish(depth_cam_info)
            # depth_img_pub.publish(br.cv2_to_imgmsg(frameDepth, encoding="bgr8"))


        if latestPacket["disp"] is not None:
            frameDisp = latestPacket["disp"].getFrame()
            maxDisparity = stereo.initialConfig.getMaxDisparity()
            # Optional, extend range 0..95 -> 0..255, for a better visualisation
            if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
            # Optional, apply false colorization
            if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_JET)
            frameDisp = np.ascontiguousarray(frameDisp)

            if debugMode == True:
                cv2.imshow(dispWindowName, frameDisp)

            disparity_img_msg.header.stamp = stamp
            right_cam_info_msg.header.stamp = stamp
            disparity_cam_pub.publish(right_cam_info_msg)
            disparity_img_pub.publish(br.cv2_to_imgmsg(frameDisp, encoding="bgr8"))

            disparity_img_compressed.header.stamp = stamp
            disparity_img_compressed.data = np.array(cv2.imencode('.jpg', frameDisp)[1]).tobytes()
            disparity_compressed_pub.publish(disparity_img_compressed)

        # Blend when both received
        if frameRight is not None and frameDisp is not None:
            # Need to have both frames in BGR format before blending

            # print(f"Right shape: {frameRight.shape}, disp shape@ {frameDisp.shape}")

            if len(frameDisp.shape) < 3:
                frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)
            
            # if detected is not None:
            #     blended = cv2.addWeighted(frameRight, 0.5, detected, 1, 0)

            blended = cv2.addWeighted(frameRight, 0.6, frameDisp, 0.4, 0)

            # cv2.imshow(blendedWindowName, blended)
            frameRight = None
            frameDisp = None

        if cv2.waitKey(1) == ord('q'):
            break