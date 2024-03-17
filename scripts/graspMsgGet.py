#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# graspMessage数据类型：id int, flag bool, type string

import rospy
from aubo_grasp.msg import graspMessage

def callback(data):
    rospy.loginfo("Received message: id={}, flag={}, type={}".format(data.id, data.flag, data.type))

def listener():
    rospy.init_node('listener_node', anonymous=True)

    rospy.Subscriber("grasp", graspMessage, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()