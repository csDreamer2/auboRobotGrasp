#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import requests
import rospy
import pyrealsense2 as rs
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import time
import json
import base64
import sys
import moveit_commander
import math


from graspnetAPI import GraspGroup
from std_msgs.msg import Bool
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from aubo_grasp.msg import Images
from cv_bridge import CvBridge
from sensor_msgs.point_cloud2 import PointCloud2
from scipy.spatial.transform import Rotation
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from data_utils import CameraInfo, create_point_cloud_from_depth_image
from aubo_grasp.msg import graspMessage
from tf.transformations import quaternion_from_euler

exe_flag = False
camera_setup_time = 2 
id=0
type="color"


class MoveItAubo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('manipulator_e5')

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo(self.end_effector_link)

        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.2)
        self.arm.set_max_velocity_scaling_factor(0.5)


    def rpy2R(self,rpy): # [r,p,y] 单位rad
        rot_x = np.array([[1, 0, 0],
                          [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                          [0, math.sin(rpy[0]), math.cos(rpy[0])]])
        rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                          [0, 1, 0],
                          [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
        rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                          [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                          [0, 0, 1]])
        R = np.dot(rot_z, np.dot(rot_y, rot_x))
        return R

    def R2rpy(self,R):
        # assert (isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def getRpy(self,rpy):
        for i in range(3):
            if rpy[i] > np.pi:
                rpy[i] -= 2 * np.pi
            elif rpy[i] < -np.pi:
                rpy[i] += 2 * np.pi
        return rpy

    def move_to_home(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

    def move_to_pose(self, gg):
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'camera_color_optical_frame'
        self.arm.set_pose_reference_frame(reference_frame)

        # 设置机械臂终端运动的目标位姿
        # 机械臂控制: 运动到指定位姿
        for i in range(20):
            rot_y = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])       
            position = gg[i].translation.tolist()
            R = np.dot(gg[i].rotation_matrix, rot_y)

            # 将欧拉角转换为四元数
            #quaternion = quaternion_from_euler(*self.getRpy(self.R2rpy(R)))
            quaternion = Rotation.from_matrix(self.rpy2R(self.getRpy(self.R2rpy(R)))).as_quat()
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            self.arm.set_pose_target(pose, self.end_effector_link)
            
            # 规划路径
            plan_success, plan, planning_time, error_code = self.arm.plan()

            # 检查是否存在有效路径
            if not plan.joint_trajectory.points:
                rospy.logwarn("No valid path found. The goal pose may be unreachable, please planning the next goal pose")
                continue
            # 存在有效路径，则继续执行
            else:
                # 初始化关节6总旋转角度
                total_joint6_rotation = 0.0

                # 遍历规划路径中的每个点
                for i in range(1, len(plan.joint_trajectory.points)):
                    # 获取当前点和上一个点的关节值
                    current_joint_values = plan.joint_trajectory.points[i].positions
                    prev_joint_values = plan.joint_trajectory.points[i - 1].positions
                    
                    # 找到关节6（通常是末端执行器的旋转关节）的索引
                    joint6_index = plan.joint_trajectory.joint_names.index('wrist3_joint')
                    
                    # 获取当前点和上一个点的关节6角度
                    current_joint6_angle = current_joint_values[joint6_index]
                    prev_joint6_angle = prev_joint_values[joint6_index]
                    
                    # 计算当前点和上一个点的关节6角度差，并累加到总旋转角度中
                    total_joint6_rotation += abs(current_joint6_angle - prev_joint6_angle)

                # 打印到达目标位置时关节6的总旋转角度
                print("Total rotation of Joint 6:", total_joint6_rotation)

                # 如果关节6的总旋转角度大于π/4小于π/2,则修改每个点的关节6角度，使得总total_joint6_rotation修改为total_joint6_rotation-π/2；
                # 如果关节6的总旋转角度大于π/2，则修改每个点的关节6角度，使得总total_joint6_rotation修改为total_joint6_rotation-π
                if total_joint6_rotation > math.pi / 4 and total_joint6_rotation < math.pi / 2:
                    rospy.loginfo("Adjusting the trajectory for total joint 6 rotation between pi/4 and pi/2...")
                    for i in range(1, len(plan.joint_trajectory.points)):
                        # 将元组转换为列表
                        current_joint_values = list(plan.joint_trajectory.points[i].positions)
                        joint6_index = plan.joint_trajectory.joint_names.index('wrist3_joint')
                        current_joint6_angle = current_joint_values[joint6_index]
                        current_joint6_angle -= math.pi / 2
                        current_joint_values[joint6_index] = current_joint6_angle
                        # 将修改后的列表重新赋值给元组
                        plan.joint_trajectory.points[i].positions = tuple(current_joint_values)
                elif total_joint6_rotation > math.pi / 2:
                    rospy.loginfo("Adjusting the trajectory for total joint 6 rotation larger than pi/2...")
                    for i in range(1, len(plan.joint_trajectory.points)):
                        # 将元组转换为列表
                        current_joint_values = list(plan.joint_trajectory.points[i].positions)
                        joint6_index = plan.joint_trajectory.joint_names.index('wrist3_joint')
                        current_joint6_angle = current_joint_values[joint6_index]
                        current_joint6_angle -= math.pi
                        current_joint_values[joint6_index] = current_joint6_angle
                        # 将修改后的列表重新赋值给元组
                        plan.joint_trajectory.points[i].positions = tuple(current_joint_values)
                    rospy.loginfo("Successfully adjusted the trajectory")
                rospy.loginfo("enter键继续,q键退出")
                key = input()
                if key == 'q':
                    rospy.loginfo("已终止抓取，相机节点启动待命...")
                    return
                else:
                    rospy.loginfo("继续执行")
                    # 控制机械臂完成运动
                    self.arm.execute(plan)

                    # 夹爪控制:关闭到指定宽度
                    width = gg[i].width * 10 + 0.05
                    if width > 0.85:
                        width = 0.85
                    elif width < 0.4:
                        width = 0.4

                    # 控制台输出夹爪宽度
                    rospy.loginfo("Gripper width: %f" % width)

                    # 控制夹爪
                    self.gripper_control(width)
                    rospy.sleep(1)

                    # 完成夹取动作后，返回'home'姿态
                    self.move_to_home()

                    # 控制夹爪松开
                    self.gripper_control(0)
                    return
        rospy.logwarn("No valid path found.")
            

    def gripper_control(self, width):
        # 初始化发布器
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

        # 创建一个新的命令
        command = outputMsg.Robotiq2FGripper_robot_output()

        # 重置夹爪
        command.rACT = 0
        self.gripper_pub.publish(command)
        rospy.sleep(0.1)  # 等待一段时间以确保命令被执行
        # 激活夹爪
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        self.gripper_pub.publish(command)
        rospy.sleep(0.1)  # 等待一段时间以确保命令被执行

        # 设置夹爪的宽度
        command.rPR = int(width * 255)
        self.gripper_pub.publish(command)

        # 等待，以确保命令被发送
        rospy.sleep(0.1)

class RealSenseImageSender:
    def __init__(self):
        # Basic settings: 启动相机，建立管道通信（耗费时间，只启动一次）
        self.pipeline = rs.pipeline()  
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)  
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)   
        self.pipe_profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color 
        self.align = rs.align(self.align_to)
        self.moveit_aubo = MoveItAubo()

        # camera params
        intrinsic = np.array([911.23583984375, 0.0, 644.1905517578125, 0.0,
                            909.9681396484375, 356.885009765625, 0.0, 0.0, 1.0]).reshape(3, 3)
        cam_depth_scale = 0.0010000000474974513
        factor_depth = 1.0 / cam_depth_scale
        self.camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
        self.num_point = 20000

        self.pointcloud_publisher = rospy.Publisher('/camera/pointcloud', PointCloud2, queue_size=10)
        self.images_publisher = rospy.Publisher('/camara/aligned_images', Images, queue_size=10)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def write_aligned_images(self):
        global exe_flag
        start_time = time.time()
        while True:
            frames = self.pipeline.wait_for_frames()   
            aligned_frames = self.align.process(frames)       

            aligned_depth_frame = aligned_frames.get_depth_frame()      
            aligned_color_frame = aligned_frames.get_color_frame()

            img_color = np.asanyarray(aligned_color_frame.get_data())
            img_depth = np.asanyarray(aligned_depth_frame.get_data())

            header = Header() 
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_color_optical_frame" 
            points= self.create_pointcloud2(img_depth, self.camera)
            pointcloud_msg = pc2.create_cloud_xyz32(header, points)

            images_msg = Images()
            images_msg.header.stamp = rospy.Time.now()
            images_msg.header.frame_id = "camera_color_optical_frame"
            images_msg.color_image = self.bridge.cv2_to_imgmsg(img_color, encoding="bgr8")
            images_msg.depth_image = self.bridge.cv2_to_imgmsg(img_depth, encoding="passthrough")

            # exe_flag: 接收到抓取指令过后开始执行抓取操作，执行完继续变为待命状态
            # time.time()-start_time >= camera_setup_time: 相机需要启动准备时间以保证图像质量
            if time.time()-start_time >= camera_setup_time:
                self.pointcloud_publisher.publish(pointcloud_msg)
                self.images_publisher.publish(images_msg)
                if exe_flag:
                    rospy.loginfo("Generating Grasping pose...")
                    self.send_images(img_color.tolist(), img_depth.tolist())
                    exe_flag = False

    def send_images(self, img_color, img_depth):
        if id==1:
            url = "http://192.168.137.1:5000/api/grasps"
            data = {"img_color": img_color, "img_depth": img_depth}
        #如果id为2，调用yolo_grasps
        elif id==2:
            url = "http://192.168.137.1:5000/api/yolo_grasps"
            data = {"img_color": img_color, "img_depth": img_depth, "type": type}
        headers = {"Content-Type": "application/json"}
        response = requests.post(url, json=data, headers=headers)
        
        if response.status_code == 200:
            content = json.loads(response.content)
            decoded_content = base64.b64decode(content["data"])
            with open("grasps.npy", "wb") as file:
                file.write(decoded_content)
        
        gg = GraspGroup().from_npy("grasps.npy") 
        self.moveit_aubo.move_to_pose(gg)
    
    def create_pointcloud2(self, depth, camera):
        cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
        mask = depth > 0
        cloud_masked = cloud[mask]
        data = cloud_masked[:, :3]
        return data
        
    def run(self):
        rospy.loginfo("相机节点启动待命...")
        self.write_aligned_images()

    def stop_pipeline(self):
        self.pipeline.stop()


def grasp_callback(msg):
    global exe_flag
    global id
    global type
    exe_flag = msg.flag
    id=msg.id
    type=msg.type

if __name__ == "__main__":
    image_sender = RealSenseImageSender()
    try:
        rospy.init_node('grasp_subscriber', anonymous=True)
        rospy.Subscriber('grasp', graspMessage, grasp_callback)
        image_sender.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        image_sender.stop_pipeline()
        pass
