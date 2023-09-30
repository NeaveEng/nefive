#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai

fps = 18

# Create pipeline
print("Creating pipeline...")
pipeline = dai.Pipeline()

print("Creating device...")
device = dai.Device()
queueNames = []

# Define sources and outputs
left = pipeline.create(dai.node.ColorCamera)
right = pipeline.create(dai.node.ColorCamera)
stereo = pipeline.create(dai.node.StereoDepth)

leftOut = pipeline.create(dai.node.XLinkOut)
rightOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)

leftOut.setStreamName("left")
rightOut.setStreamName("right")
disparityOut.setStreamName("disp")

queueNames.append("left")
queueNames.append("right")
queueNames.append("disp")

# The disparity is computed at this resolution, then upscaled to RGB resolution
colorResolution = dai.ColorCameraProperties.SensorResolution.THE_1200_P

left.setResolution(colorResolution)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
left.setIspScale(1,3)
left.setFps(fps)

right.setResolution(colorResolution)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
right.setIspScale(1,3)
right.setFps(fps)

stereo.initialConfig.setConfidenceThreshold(245)
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)  # KERNEL_7x7 default
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
stereo.setExtendedDisparity(False)
stereo.setSubpixel(True)



print("Creating links...")    
# Linking
left.isp.link(stereo.left)
left.isp.link(leftOut.input)
right.isp.link(stereo.right)
right.isp.link(rightOut.input)
stereo.disparity.link(disparityOut.input)

# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)

    print("Pipeline started.")
    frameLeft = None
    frameRight = None
    frameDisp = None

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    rightWindowName = "right"
    cv2.namedWindow(rightWindowName)
    
    leftWindowName = "left"
    cv2.namedWindow(leftWindowName)

    dispWindowName = "disp"
    cv2.namedWindow(dispWindowName)

    while True:
        latestPacket = {}
        latestPacket["left"] = None
        latestPacket["right"] = None
        latestPacket["disp"] = None

        queueEvents = device.getQueueEvents(("left", "right", "disp"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["left"] is not None:
            frameLeft = latestPacket["left"].getCvFrame()
            cv2.imshow(leftWindowName, frameLeft)

        if latestPacket["right"] is not None:
            frameRight = latestPacket["right"].getCvFrame()
            cv2.imshow(rightWindowName, frameRight)

        if latestPacket["disp"] is not None:
            frameDisp = latestPacket["disp"].getCvFrame()

            maxDisparity = stereo.initialConfig.getMaxDisparity()
            # Optional, extend range 0..95 -> 0..255, for a better visualisation
            if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
            # Optional, apply false colorization
            if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_JET)
            frameDisp = np.ascontiguousarray(frameDisp)
            # croppedFrameDisp = frameDisp[0:int(h/2), 0:int(w/2), ]
            # frameDisp = cv2.resize(croppedFrameDisp, dsize=(960,600), interpolation=cv2.INTER_CUBIC)
            cv2.imshow(dispWindowName, frameDisp)


        if cv2.waitKey(1) == ord('q'):
            break
