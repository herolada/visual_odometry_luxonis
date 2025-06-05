#!/usr/bin/env python3

from copy import deepcopy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_system_default
from rerun_node import RerunNode
from tf_transformations import quaternion_from_matrix
import cv2
import depthai as dai
import numpy as np
import rclpy
import time

# Create pipeline

with dai.Pipeline() as p:
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getMxId()} {device.state}")
        
    fps = 10
    width = 1920//4
    height = 1200//4
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    featureTracker1 = p.create(dai.node.FeatureTracker)
    # featureTracker2 = p.create(dai.node.FeatureTracker)
    odom = p.create(dai.node.RTABMapVIO)

    manip_l = p.create(dai.node.ImageManip)
    manip_l.initialConfig.setResize(width,height)
    manip_r = p.create(dai.node.ImageManip)
    manip_r.initialConfig.setResize(width,height)

    rerunViewer = RerunNode()
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    featureTracker1.setHardwareResources(1,2)
    featureTracker1.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTracker1.initialConfig.setNumTargetFeatures(1000)
    featureTracker1.initialConfig.setMotionEstimator(False)
    featureTracker1.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
    # featureTracker2.setHardwareResources(1,1)
    # featureTracker2.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    # featureTracker2.initialConfig.setNumTargetFeatures(1000)
    # featureTracker2.initialConfig.setMotionEstimator(False)
    # featureTracker2.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
    left.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_1200_P)
    left.setFps(fps)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_1200_P)
    right.setFps(fps)

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)

    
    #stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7) # you can filter like this!!
    # stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

    # Linking

    left.out.link(manip_l.inputImage)
    manip_l.out.link(stereo.left)
    right.out.link(manip_r.inputImage)
    manip_r.out.link(stereo.right)
    stereo.rectifiedLeft.link(featureTracker1.inputImage)
    # left.out.link(featureTracker1.inputImage)
    # stereo.rectifiedLeft.link(featureTracker2.inputImage)
    
    featureTracker1.passthroughInputImage.link(odom.rect)
    stereo.depth.link(odom.depth)
    featureTracker1.outputFeatures.link(odom.features)
    imu.out.link(odom.imu)
    # odom.passthroughDepth.link(rerunViewer.inputImg)
    # odom.transform.link(rerunViewer.inputTrans)

    q_l = p.getNode(7).out.createOutputQueue()
    q_r = p.getNode(8).out.createOutputQueue()
    q = p.getNode(3).disparity.createOutputQueue()
    q_feat_1 = p.getNode(4).outputFeatures.createOutputQueue()
    # q_feat_2 = p.getNode(5).outputFeatures.createOutputQueue()

    p.start()
    while p.isRunning():
        inDisparity = q.get()  # blocking call, will wait until a new data has arrived
        frame = inDisparity.getFrame()
        # Normalization for better visualization
        frame = (frame * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)

        cv2.imshow("disparity", frame)

        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        cv2.imshow("disparity_color", frame)

        # if cv2.waitKey(1) == ord('q'):
        #     break
        left_img = q_l.get().getFrame()
        right_img = q_r.get().getFrame()

        cv2.imshow("l", left_img)
        cv2.imshow("r", right_img)

        if cv2.waitKey(1) == ord('q'):
            break