#!/usr/bin/env python3
import std_msgs.msg
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
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
# from projector_device import PointCloudVisualizer
# from projector_3d import PointCloudVisualizer3d
# import open3d_ros_helper as orh
from pathlib import Path
import os
import rospkg


# se3 = np.eye(4)
# ros_transform = orh.se3_to_transform(se3) 

debugMode = False

rospack = rospkg.RosPack()
pkgPath = rospack.get_path('nefive_vision')

# StereoDepth config options.
# whether or not to align the depth image on host (As opposed to on device), only matters if align_depth = True
lrcheck = True  # Better handling for occlusions
extended = True  # Closer-in minimum depth, disparity range is doubled
subpixel = False  # True  # Better accuracy for longer distance, fractional disparity 32-levels

def signal_handler(signal, frame):
    print('CTRL-C caught, exiting.')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)



def open3d_to_ros(open3d_cloud, frame_id="base_link"):
    # Convert the cloud to a numpy array
    xyz_array = np.asarray(open3d_cloud.points, dtype=np.float32)

    # Create a header for the PointCloud2 message
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Create the PointCloud2 message
    ros_cloud = pc2.create_cloud_xyz32(header, xyz_array)

    return ros_cloud

def create_xyz(width, height, camera_matrix):
    xs = np.linspace(0, width - 1, width, dtype=float6)
    ys = np.linspace(0, height - 1, height, dtype=float)

    # generate grid by stacking coordinates
    base_grid = np.stack(np.meshgrid(xs, ys)) # WxHx2
    points_2d = base_grid.transpose(1, 2, 0) # 1xHxWx2

    # unpack coordinates
    u_coord: np.array = points_2d[..., 0]
    v_coord: np.array = points_2d[..., 1]

    # unpack intrinsics
    fx: np.array = camera_matrix[0, 0]
    fy: np.array = camera_matrix[1, 1]
    cx: np.array = camera_matrix[0, 2]
    cy: np.array = camera_matrix[1, 2]

    # projective
    x_coord: np.array = (u_coord - cx) / fx
    y_coord: np.array = (v_coord - cy) / fy

    xyz = np.stack([x_coord, y_coord], axis=-1)
    return np.pad(xyz, ((0,0),(0,0),(0,1)), "constant", constant_values=1.0)

def getPath(resolution):
    (width, heigth) = resolution
    path = Path(pkgPath, "models", "out")
    path.mkdir(parents=True, exist_ok=True)
    name = f"pointcloud_{width}x{heigth}"

    return_path = str(path / (name + '.blob'))
    if os.path.exists(return_path):
        print(f"{return_path} exists!")
        return return_path
    
    print(f"{return_path} doesn't exists!")


    # Model doesn't exist, create it
    import models.depth_to_3d
    return models.depth_to_3d.createBlob(resolution, path, name)

def configureDepthPostProcessing(stereoDepthNode):
    """
    In-place post-processing configuration for a stereo depth node
    The best combo of filters is application specific. Hard to say there is a one size fits all.
    They also are not free. Even though they happen on device, you pay a penalty in fps.
    """
    stereoDepthNode.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

    # stereoDepthNode.initialConfig.setBilateralFilterSigma(16)
    config = stereoDepthNode.initialConfig.get()
    config.postProcessing.speckleFilter.enable = True
    config.postProcessing.speckleFilter.speckleRange = 60
    config.postProcessing.temporalFilter.enable = True

    config.postProcessing.spatialFilter.holeFillingRadius = 2
    config.postProcessing.spatialFilter.numIterations = 1
    config.postProcessing.thresholdFilter.minRange = 700  # mm
    config.postProcessing.thresholdFilter.maxRange = 4000  # mm
    # config.postProcessing.decimationFilter.decimationFactor = 1
    config.censusTransform.enableMeanMode = True
    config.costMatching.linearEquationParameters.alpha = 0
    config.costMatching.linearEquationParameters.beta = 2

    depth.initialConfig.setDepthUnit(depth.initialConfig.AlgorithmControl.DepthUnit.METER)

    stereoDepthNode.initialConfig.set(config)
    stereoDepthNode.setLeftRightCheck(lrcheck)
    stereoDepthNode.setExtendedDisparity(extended)
    stereoDepthNode.setSubpixel(subpixel)
    stereoDepthNode.setRectifyEdgeFillColor(0)  # Black, to better see the cutout

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
leftOut = pipeline.create(dai.node.XLinkOut)
leftOut.setStreamName("left")
queueNames.append("left")

rightCam = pipeline.create(dai.node.ColorCamera)
rightOut = pipeline.create(dai.node.XLinkOut)
rightOut.setStreamName("right")
queueNames.append("right")

centerCam = pipeline.create(dai.node.ColorCamera)
centerOut = pipeline.create(dai.node.XLinkOut)
centerOut.setStreamName("center")
queueNames.append("center")

depth = pipeline.create(dai.node.StereoDepth)
depthOut = pipeline.create(dai.node.XLinkOut)
depthOut.setStreamName("depth")
queueNames.append("depth")

disparityOut = pipeline.create(dai.node.XLinkOut)
disparityOut.setStreamName("disp")
queueNames.append("disp")

pointcloud: dai.node.PointCloud = pipeline.create(dai.node.PointCloud)
pointcloudOut = pipeline.create(dai.node.XLinkOut)
pointcloudOut.setStreamName("pcl")
pointcloudOut.input.setBlocking(False)
queueNames.append("pcl")

sync = pipeline.create(dai.node.Sync)


scaleNumerator   = 1
scaleDenominator = 3
height = 1200 * (scaleNumerator / scaleDenominator)
width = 1920 * (scaleNumerator / scaleDenominator)

# The disparity is computed at this resolution, then upscaled to RGB resolution
colorResolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P
stereoColorResolution = dai.ColorCameraProperties.SensorResolution.THE_1200_P

leftCam.setResolution(stereoColorResolution)
leftCam.setBoardSocket(dai.CameraBoardSocket.CAM_B)
leftCam.setIspScale(scaleNumerator, scaleDenominator) 
leftCam.setFps(fps)

rightCam.setResolution(stereoColorResolution)
rightCam.setBoardSocket(dai.CameraBoardSocket.CAM_C)
rightCam.setIspScale(scaleNumerator, scaleDenominator)
rightCam.setFps(fps)

centerCam.setResolution(colorResolution)
centerCam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
centerCam.setIspScale(scaleNumerator, scaleDenominator)
centerCam.setFps(25)

depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(True)
depth.setExtendedDisparity(False)
depth.setSubpixel(True)
depth.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
config = depth.initialConfig.get()
config.postProcessing.thresholdFilter.minRange = 200
config.postProcessing.thresholdFilter.maxRange = 1000
depth.initialConfig.set(config)

print("Aligning to right camera...")
depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)

# depth.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)

configureDepthPostProcessing(depth)

print("Creating links...")
# Linking
leftCam.video.link(leftOut.input)
rightCam.video.link(rightOut.input)
centerCam.isp.link(centerOut.input)

leftCam.isp.link(depth.left)
rightCam.isp.link(depth.right)

depth.depth.link(depthOut.input)
depth.depth.link(pointcloud.inputDepth)
depth.disparity.link(disparityOut.input)

centerCam.isp.link(sync.inputs["center"])
pointcloud.outputPointCloud.link(sync.inputs["pcl"])
pointcloud.initialConfig.setSparse(False)
sync.out.link(pointcloudOut.input)

inConfig = pipeline.create(dai.node.XLinkIn)
inConfig.setStreamName("config")
inConfig.out.link(pointcloud.inputConfig)

left_img_pub = rospy.Publisher('camera/left/image', Image, queue_size=1)
right_img_pub = rospy.Publisher('camera/right/image', Image, queue_size=1)
center_img_pub = rospy.Publisher('camera/center/image', Image, queue_size=1)
depth_img_pub = rospy.Publisher('camera/depth/image', Image, queue_size=1)
disparity_img_pub = rospy.Publisher('camera/disparity/image', Image, queue_size=1)

left_cam_pub = rospy.Publisher('camera/right/camera_info', CameraInfo, queue_size=1)
right_cam_pub = rospy.Publisher('camera/left/camera_info', CameraInfo, queue_size=1)
center_cam_pub = rospy.Publisher('camera/center/camera_info', CameraInfo, queue_size=1)
depth_cam_pub = rospy.Publisher('camera/depth/camera_info', CameraInfo, queue_size=1)
disparity_cam_pub = rospy.Publisher('camera/disparity/camera_info', CameraInfo, queue_size=1)

left_compressed_pub = rospy.Publisher('camera/left/image/compressed', CompressedImage, queue_size=1)
right_compressed_pub = rospy.Publisher('camera/right/image/compressed', CompressedImage, queue_size=1)
center_compressed_pub = rospy.Publisher('camera/center/image/compressed', CompressedImage, queue_size=1)
disparity_compressed_pub = rospy.Publisher('camera/disparity/image/compressed', CompressedImage, queue_size=1)

pcl2_pub = rospy.Publisher('camera/depth/points', PointCloud2, queue_size=1)

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

center_img_msg = Image()
center_img_msg.height = height
center_img_msg.width = width
center_img_msg.step = width * 3
center_img_msg.encoding = 'rgb8'
center_img_msg.header.frame_id = 'center_camera'

depth_img_msg = Image()
depth_img_msg.height = height
depth_img_msg.width = width
depth_img_msg.step = width * 1  # bytes per row: pixels * channels * bytes per channel (1 normally)
depth_img_msg.encoding = 'uint16'
depth_img_msg.header.frame_id = 'right_camera'  # TF frame

disparity_img_msg = Image()
disparity_img_msg.height = height
disparity_img_msg.width = width
disparity_img_msg.step = width * 3
disparity_img_msg.encoding = 'rgb8'
disparity_img_msg.header.frame_id = 'right_camera'

left_img_compressed = CompressedImage()
right_img_compressed = CompressedImage()
center_img_compressed = CompressedImage()
disparity_img_compressed = CompressedImage()

left_img_compressed.format = "jpeg"
right_img_compressed.format = "jpeg"
center_img_compressed.format = "jpeg"
disparity_img_compressed.format = "jpeg"

# parse the left and right camera calibration yaml files
left_cam_info = rospy.get_param("left_cam_info")
right_cam_info = depth_cam_info = disparity_cam_info = rospy.get_param("right_cam_info")
# center_cam_info = rospy.get_param("center_cam_info")
left_cam_info_msg = parse_calibration_yaml(left_cam_info)
right_cam_info_msg = parse_calibration_yaml(right_cam_info)
# center_cam_info_msg = parse_calibration_yaml(center_cam_info)

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
    framePcl = None

    resolution = (640,400)

    calibData = device.readCalibration()
    M_right = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C,
        dai.Size2f(resolution[0], resolution[1]),
    )

    if debugMode == True:
        # Configure windows; trackbar adjusts blending ratio of rgb/depth
        leftWindowName = "left"
        rightWindowName = "right"
        centerWindowName = "center"
        depthWindowName = "depth"
        dispWindowName = "disp"

        cv2.namedWindow(leftWindowName)
        cv2.namedWindow(rightWindowName)
        cv2.namedWindow(centerWindowName)
        cv2.namedWindow(depthWindowName)
        cv2.namedWindow(dispWindowName)


    R_camera_to_world = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).astype(float)
    pcd = o3d.geometry.PointCloud()

    while not rospy.is_shutdown():
        latestPacket = {}
        latestPacket["left"] = None
        latestPacket["right"] = None
        latestPacket["center"] = None
        latestPacket["depth"] = None
        latestPacket["disp"] = None
        latestPacket["pcl"] = None
        frameCenter = None

        stamp = rospy.Time.now()
        left_cam_info_msg.header.stamp = stamp
        right_cam_info_msg.header.stamp = stamp

        queueEvents = device.getQueueEvents(("left", "right", "center", "depth", "disp", "pcl"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["center"] is not None:
            frameCenter = latestPacket["center"].getCvFrame()
            if debugMode == True:
                cv2.imshow(centerWindowName, frameCenter)

            center_img_msg.header.stamp = stamp
            # center_cam_pub.publish(right_cam_info_msg)
            center_img_pub.publish(br.cv2_to_imgmsg(frameCenter, encoding="bgr8"))

            center_img_compressed.header.stamp = stamp
            center_img_compressed.data = np.array(cv2.imencode('.jpg', frameCenter)[1]).tobytes()
            center_compressed_pub.publish(center_img_compressed)


        if latestPacket["pcl"] is not None:
            # Use this as an example to visualise with Open3d instead:
            # https://github.com/luxonis/depthai-experiments/blob/d6a1dafb988e74e42f3250c71ca9905a6fe44b0e/gen2-pointcloud/device-pointcloud/main.py#L174C16-L174C16
            # viewer.log_points("Pointcloud", pcl_arr.reshape(-1, 3), colors=colors.reshape(-1, 3))
            # viewer.log_image("Color", colors)

            # if debugMode == True:
            #     pcl_converter.visualize_pcd()
            
            points = latestPacket["pcl"]["pcl"].getPoints().astype(np.float64)
            
            pcd.points = o3d.utility.Vector3dVector(points)
            # if(frameCenter is not None):
            #     colors = (frameCenter.reshape(-1, 3) / 255.0).astype(np.float64)
            #     pcd.colors = o3d.utility.Vector3dVector(colors)

            # print(f"Pointcloud has {len(pcd)} points")

            pcl_msg = open3d_to_ros(pcd, frame_id="right_camera")
            pcl2_pub.publish(pcl_msg)

            
        if latestPacket["left"] is not None:
            frameLeft = latestPacket["left"].getCvFrame()
            if debugMode == True:
                cv2.imshow(leftWindowName, frameLeft)

            left_img_msg.header.stamp = stamp
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
            right_cam_pub.publish(right_cam_info_msg)
            right_img_pub.publish(br.cv2_to_imgmsg(frameRight, encoding="bgr8"))

            right_img_compressed.header.stamp = stamp
            right_img_compressed.data = np.array(cv2.imencode('.jpg', frameRight)[1]).tobytes()
            right_compressed_pub.publish(right_img_compressed)

        if latestPacket["depth"] is not None:
            frameDepth = latestPacket["depth"].getCvFrame()
            if debugMode == True:
                cv2.imshow(depthWindowName, frameDepth)

            depth_img_msg.header.stamp = stamp
            depth_cam_pub.publish(right_cam_info_msg)
            depth_img_pub.publish(br.cv2_to_imgmsg(frameDepth))

        if latestPacket["disp"] is not None:
            frameDisp = latestPacket["disp"].getFrame()
            maxDisparity = depth.initialConfig.getMaxDisparity()
            # Optional, extend range 0..95 -> 0..255, for a better visualisation
            if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
            # Optional, apply false colorization
            if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_JET)
            frameDisp = np.ascontiguousarray(frameDisp)

            if debugMode == True:
                cv2.imshow(dispWindowName, frameDisp)

            disparity_img_msg.header.stamp = stamp
            disparity_cam_pub.publish(right_cam_info_msg)
            disparity_img_pub.publish(br.cv2_to_imgmsg(frameDisp, encoding="rgb8"))

            disparity_img_compressed.header.stamp = stamp
            disparity_img_compressed.data = np.array(cv2.imencode('.jpg', frameDisp)[1]).tobytes()
            disparity_compressed_pub.publish(disparity_img_compressed)

        # if (frameDepth is not None) and (frameRight is not None):
        #     pcl = pcl_converter_3d.rgbd_to_projection(frameDepth, frameRight)

        #     # Rotate the pointcloud such that it is in the world coordinate frame (easier to visualize)
        #     pcl.rotate(R_camera_to_world, center=np.array([0,0,0],dtype=np.float64))

        #     pcl_msg = orh.o3dpc_to_rospc(pcl, 'right_camera', stamp)
        #     pcl2_pub.publish(pcl_msg)

        #     if debugMode == True:
        #         pcl_converter_3d.visualize_pcd()

        if cv2.waitKey(1) == ord('q'):
            break