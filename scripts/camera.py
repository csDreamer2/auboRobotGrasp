#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import pyrealsense2 as rs
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import time

from aubo_grasp.msg import Images
from std_msgs.msg import Header
from sensor_msgs.point_cloud2 import PointCloud2
from cv_bridge import CvBridge
from data_utils import CameraInfo, create_point_cloud_from_depth_image


# basic settings
pipeline = rs.pipeline()  
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)  
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)   

pipe_profile = pipeline.start(config)

align_to = rs.stream.color 
align = rs.align(align_to)

# camera params
intrinsic = np.array([911.23583984375, 0.0, 644.1905517578125, 0.0,
                       909.9681396484375, 356.885009765625, 0.0, 0.0, 1.0]).reshape(3, 3)
cam_depth_scale = 0.0010000000474974513
factor_depth = 1.0 / cam_depth_scale
camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
num_point = 20000 

# Initialize ROS Nodes
rospy.init_node('camera_publisher', anonymous=True)

images_publisher = rospy.Publisher('/camara/aligned_images', Images, queue_size=10)
images_msg = Images()

pointcloud_publisher = rospy.Publisher('/camara/pointcloud', PointCloud2, queue_size=10)
pointcloud_msg = PointCloud2()

# Initialize CvBridge
bridge = CvBridge()
          
def publish_data():
    start_time = time.time()
    try:
        while True:
            frames = pipeline.wait_for_frames()   
            aligned_frames = align.process(frames)       

            aligned_depth_frame = aligned_frames.get_depth_frame()      
            aligned_color_frame = aligned_frames.get_color_frame()

            img_color = np.asanyarray(aligned_color_frame.get_data())
            img_depth = np.asanyarray(aligned_depth_frame.get_data())
            
            images_msg.header.stamp = rospy.Time.now()
            images_msg.color_image = bridge.cv2_to_imgmsg(img_color, encoding="bgr8")
            images_msg.depth_image = bridge.cv2_to_imgmsg(img_depth, encoding="passthrough")

            if time.time()-start_time >= 2:
                rospy.loginfo("发布图片数据")
                images_publisher.publish(images_msg)

            pointcloud_msg.header.stamp = rospy.Time.now()
            pointcloud_msg.header.frame_id = "camera_color_optical_frame" 
            pointcloud_msg.fields, pointcloud_msg.data = create_pointcloud2(img_color, img_depth)

            if time.time()-start_time >= 2:
                rospy.loginfo("发布点云数据")
                pointcloud_publisher.publish(pointcloud_msg)
            
            rospy.sleep(1)
    finally:
        pipeline.stop()

def create_pointcloud2(color, depth):
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
    mask = depth > 0
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    fields = [
    pc2.PointField(name="x", offset=0, datatype=7, count=1),
    pc2.PointField(name="y", offset=4, datatype=7, count=1),
    pc2.PointField(name="z", offset=8, datatype=7, count=1),
    pc2.PointField(name="rgb", offset=12, datatype=7, count=1),
    ]

    rgb_data = ((color_masked[:, 0] * 255).astype(np.uint32) << 16) | \
           ((color_masked[:, 1] * 255).astype(np.uint32) << 8) | \
           (color_masked[:, 2] * 255).astype(np.uint32)
    
    data = list(np.column_stack((cloud_masked, rgb_data)))
    rospy.loginfo(data)
    return fields, data


if __name__ == "__main__":
    publish_data()
