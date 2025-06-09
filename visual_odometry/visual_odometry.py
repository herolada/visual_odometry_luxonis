#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import time
import depthai as dai
from rerun_node import RerunNode
from rclpy.qos import QoSProfile, qos_profile_system_default
import numpy as np
from tf_transformations import quaternion_from_matrix
from copy import deepcopy

import time
import depthai as dai
from rerun_node import RerunNode

# Create pipeline

# with dai.Pipeline() as p:
#     for device in dai.Device.getAllAvailableDevices():
#         print(f"{device.getMxId()} {device.state}")
        
#     fps = 20
#     width = 1920//4
#     height = 1200//4
#     # Define sources and outputs
#     left = p.create(dai.node.MonoCamera)
#     right = p.create(dai.node.MonoCamera)
#     imu = p.create(dai.node.IMU)
#     stereo = p.create(dai.node.StereoDepth)
#     featureTracker1 = p.create(dai.node.FeatureTracker)
#     # featureTracker2 = p.create(dai.node.FeatureTracker)
#     odom = p.create(dai.node.RTABMapVIO)

#     manip_l = p.create(dai.node.ImageManip)
#     manip_l.initialConfig.setResize(width,height)
#     manip_r = p.create(dai.node.ImageManip)
#     manip_r.initialConfig.setResize(width,height)

#     rerunViewer = RerunNode()
#     imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
#     imu.setBatchReportThreshold(1)
#     imu.setMaxBatchReports(10)

#     featureTracker1.setHardwareResources(2,2)
#     featureTracker1.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
#     featureTracker1.initialConfig.setNumTargetFeatures(1000)
#     featureTracker1.initialConfig.setMotionEstimator(False)
#     featureTracker1.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
#     # featureTracker2.setHardwareResources(1,1)
#     # featureTracker2.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
#     # featureTracker2.initialConfig.setNumTargetFeatures(1000)
#     # featureTracker2.initialConfig.setMotionEstimator(False)
#     # featureTracker2.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
#     # left.setCamera("left")
#     left.setBoardSocket(dai.CameraBoardSocket.CAM_A)
#     # left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
#     left.setFps(fps)
#     # right.setCamera("right")
#     right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
#     # right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
#     right.setFps(fps)

#     stereo.setExtendedDisparity(False)
#     stereo.setLeftRightCheck(True)
#     stereo.setRectifyEdgeFillColor(0)
#     stereo.enableDistortionCorrection(True)
#     stereo.initialConfig.setLeftRightCheckThreshold(10)
#     stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
#     #stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7) # you can filter like this!!
#     # stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

#     # Linking

#     left.out.link(manip_l.inputImage)
#     manip_l.out.link(stereo.left)
#     right.out.link(manip_r.inputImage)
#     manip_r.out.link(stereo.right)
#     stereo.rectifiedLeft.link(featureTracker1.inputImage)
#     # left.out.link(featureTracker1.inputImage)
#     # stereo.rectifiedLeft.link(featureTracker2.inputImage)
    
#     featureTracker1.passthroughInputImage.link(odom.rect)
#     stereo.depth.link(odom.depth)
#     featureTracker1.outputFeatures.link(odom.features)
#     imu.out.link(odom.imu)
#     odom.passthroughDepth.link(rerunViewer.inputImg)
#     odom.transform.link(rerunViewer.inputTrans)

#     q_stereo = p.getNode(3).depth.createOutputQueue()
#     q_feat_1 = p.getNode(4).outputFeatures.createOutputQueue()
#     # q_feat_2 = p.getNode(5).outputFeatures.createOutputQueue()

#     p.start()
#     while p.isRunning():
#         time.sleep(1)
#         # print(f"Num features {len(q_feat_1.get().trackedFeatures)}")
#         # q_stereo.get()



#         # tracked_feats_1 = q_feat_1.get()
#         # for f in tracked_feats_1.trackedFeatures:
#         #     f.id += 10000000
#         # tracked_feats_2 = q_feat_2.get()

class RTABMapVIONode(Node):

    def __init__(self):
        super().__init__('rtabmap_vio')

        self.declare_parameter('device_id', '14442C10B11BEFCF00')
        self.device_id_ = self.get_parameter('device_id').get_parameter_value().string_value

        self.declare_parameter('rescale', True)
        self.rescale_ = self.get_parameter('rescale').get_parameter_value().bool_value

        self.declare_parameter('width', 1920//2)
        self.width_ = self.get_parameter('width').get_parameter_value().integer_value

        self.declare_parameter('height', 1200//2)
        self.height_ = self.get_parameter('height').get_parameter_value().integer_value

        self.declare_parameter('fps', 10.)
        self.fps_ = self.get_parameter('fps').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Odometry, '/odom/visual', qos_profile_system_default)

        timer_period = 1/self.fps_  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.msg = Odometry()
        self.prev_msg = Odometry()

        print("Available devices:")
        for device in dai.Device.getAllAvailableDevices():
            print(f"{device.getDeviceId()} {device.state}")

        # 14442C102133AECF00 
        # 14442C108101EDCF00 
        # 14442C10418237D200 
        # 14442C10B11BEFCF00 

        self.device_info = dai.DeviceInfo(self.device_id_)
        self.device = dai.Device(self.device_info)

        self.p = dai.Pipeline(self.device)
        # Define sources and outputs
        self.left = self.p.create(dai.node.MonoCamera)
        self.right = self.p.create(dai.node.MonoCamera)
        self.imu = self.p.create(dai.node.IMU)
        self.stereo = self.p.create(dai.node.StereoDepth)
        self.featureTracker = self.p.create(dai.node.FeatureTracker)
        self.odom = self.p.create(dai.node.RTABMapVIO)  # TODO initialize with best odom (e.g. gps_odom)

        if self.rescale_:
            self.manip_l = self.p.create(dai.node.ImageManip)
            self.manip_l.initialConfig.setResize(self.width_, self.height_)
            self.manip_r = self.p.create(dai.node.ImageManip)
            self.manip_r.initialConfig.setResize(self.width_, self.height_)

        # rerunViewer = RerunNode()
        self.imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        self.imu.setBatchReportThreshold(1)
        self.imu.setMaxBatchReports(10)

        self.featureTracker.setHardwareResources(1,2)
        self.featureTracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
        self.featureTracker.initialConfig.setNumTargetFeatures(1000)
        self.featureTracker.initialConfig.setMotionEstimator(False)
        self.featureTracker.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
        self.left.setCamera("left")
        self.left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.left.setFps(self.fps_)
        self.right.setCamera("right")
        self.right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.right.setFps(self.fps_)

        self.stereo.setExtendedDisparity(False)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setRectifyEdgeFillColor(0)
        self.stereo.enableDistortionCorrection(True)
        self.stereo.initialConfig.setLeftRightCheckThreshold(10)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

        # Linking
        
        if self.rescale_:
            self.left.out.link(self.manip_l.inputImage)
            self.manip_l.out.link(self.stereo.left)
            self.right.out.link(self.manip_r.inputImage)
            self.manip_r.out.link(self.stereo.right)

            self.stereo.rectifiedLeft.link(self.featureTracker.inputImage)
            self.featureTracker.passthroughInputImage.link(self.odom.rect)
            self.stereo.depth.link(self.odom.depth)
            self.featureTracker.outputFeatures.link(self.odom.features)
            self.imu.out.link(self.odom.imu)
        else:
            self.left.out.link(self.stereo.left)
            self.right.out.link(self.stereo.right)
            self.stereo.rectifiedLeft.link(self.featureTracker.inputImage)
            self.featureTracker.passthroughInputImage.link(self.odom.rect)
            self.stereo.depth.link(self.odom.depth)
            self.featureTracker.outputFeatures.link(self.odom.features)
            self.imu.out.link(self.odom.imu)
            # self.odom.passthroughRect.link(rerunViewer.inputImg)
            # self.odom.transform.link(rerunViewer.inputTrans)

        self.q_cam = self.p.getNode(0).out.createOutputQueue(maxSize=10,blocking=False)
        self.q_odom = self.p.getNode(5).transform.createOutputQueue(maxSize=10,blocking=False)
        self.p.start()

    def build_odometry_msg(self, data, ts):
        
        # Fill header
        self.msg.header.stamp.sec = ts.seconds #data.getTimestamp().seconds
        self.msg.header.stamp.nanosec = ts.microseconds * 1000 #data.getTimestamp().microseconds * 1000
        #msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = "odom"
        self.msg.child_frame_id = "camera_front"

        quat = data.getQuaternion()
        translation = data.getTranslation()

        # Position
        self.msg.pose.pose.position.x = translation.x
        self.msg.pose.pose.position.y = translation.y
        self.msg.pose.pose.position.z = translation.z

        self.msg.pose.pose.orientation.x = quat.qx
        self.msg.pose.pose.orientation.y = quat.qy
        self.msg.pose.pose.orientation.z = quat.qz
        self.msg.pose.pose.orientation.w = quat.qw

    def timer_callback(self):
        if self.p.isRunning():
            odom_data = self.q_odom.get()
            ts = self.q_cam.get().getTimestamp()    # TODO: not good solution
            self.build_odometry_msg(odom_data,ts)
            print(ts)
            
            if self.prev_msg.pose == self.msg.pose:
                self.odom.reset(self.q_odom.get())  # TODO reset with best odometry (e.g. gps_odom)
                print("RESETING ODOM")

            # if transform != [0.,0.,0.,0.5,0.5,0.5,0.5]:
            #     last_good_transform = q_odom.get()
            
            # prev_transform = transform

            self.prev_msg = deepcopy(self.msg)
            self.publisher_.publish(self.msg)
        else:
            print("DEPTHAI PIPELINE NOT RUNNING")

    def destroy_node(self):
        self.p.__exit__()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RTABMapVIONode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()