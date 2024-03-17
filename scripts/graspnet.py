#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import requests
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
import time
import json
import base64

from graspnetAPI import GraspGroup
from grasp import MoveItAubo
from cv_bridge import CvBridge
from msg import Images

img_color = None
img_depth = None

def images_callback(msg):
    global img_color, img_depth
    bridge = CvBridge()
    img_color = bridge.imgmsg_to_cv2(msg.color_image, desired_encoding="bgr8")
    img_depth = bridge.imgmsg_to_cv2(msg.depth_image, desired_encoding="passthrough")

def get_grasps():
    global img_color, img_depth
    url = "http://172.18.10.101:5000/api/grasps"
    data=  {"img_color": img_color,"img_depth": img_depth}
    headers = {"Content-Type": "application/json"}
    response = requests.post(url, json=data, headers=headers)
    if response.status_code == 200:
        content = json.loads(response.content)
        decoded_content = base64.b64decode(content["data"])
    with open("grasps.npy", "wb") as file:
        file.write(decoded_content)
    gg = GraspGroup().from_npy("grasps.npy")   
    return gg

if __name__ == "__main__":
    try:
        rospy.init_node('image_subscriber', anonymous=True)
        rospy.Subscriber('/camera/aligned_images', Images, images_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


