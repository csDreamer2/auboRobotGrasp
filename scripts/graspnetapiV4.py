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
import tf

from tf import TransformListener
from geometry_msgs.msg import PoseStamped
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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import Plane, SolidPrimitive
from geometry_msgs.msg import Pose, Vector3

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint

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

        #设置规划时间
        self.arm.set_planning_time(5)

        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

        self.arm.set_planner_id("TRRT")

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)


    def move_to_home(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
    
    def getRpy(self,rpy):
        for i in range(3):
            if rpy[i] > np.pi:
                rpy[i] -= 2 * np.pi
            elif rpy[i] < -np.pi:
                rpy[i] += 2 * np.pi
        return rpy  
    
    def transPose(self, pose):

        from_link = '/camera_color_optical_frame'
        to_link = '/base_link'
        
        tfl = TransformListener()
        
        rospy.sleep(0.5)
    
        time = rospy.Time(0)
        mpose = PoseStamped()
        
        mpose.pose.position.x = pose.position.x
        mpose.pose.position.y = pose.position.y
        mpose.pose.position.z = pose.position.z
        
        mpose.pose.orientation.x = pose.orientation.x
        mpose.pose.orientation.y = pose.orientation.y
        mpose.pose.orientation.z = pose.orientation.z
        mpose.pose.orientation.w = pose.orientation.w
        
        mpose.header.frame_id = from_link
        mpose.header.stamp = rospy.Time.now()

        rospy.sleep(0.5)
        mpose_transf = None
        
        tfl.waitForTransform(to_link,from_link,time,rospy.Duration(5.0))
        
        if tfl.canTransform(to_link,from_link,time):
        
            mpose_transf = tfl.transformPose(to_link,mpose)
            base_pose=Pose()
            base_pose.position.x = mpose_transf.pose.position.x
            base_pose.position.y = mpose_transf.pose.position.y
            base_pose.position.z = mpose_transf.pose.position.z
            base_pose.orientation.x = mpose_transf.pose.orientation.x
            base_pose.orientation.y = mpose_transf.pose.orientation.y
            base_pose.orientation.z = mpose_transf.pose.orientation.z
            base_pose.orientation.w = mpose_transf.pose.orientation.w
            return base_pose
    
        else:
            return None

    def move_to_pose(self, gg):
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(reference_frame)

        # 设置机械臂终端运动的目标位姿
        # 机械臂控制: 运动到指定位姿
        for i in range(len(gg)):
            rot_y = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])  
            if not gg:
                rospy.logwarn("Grasp group is empty. Unable to proceed with grasping.")
                return
            position = gg[i].translation.tolist()
            quaternion = Rotation.from_matrix(np.dot(gg[i].rotation_matrix, rot_y)).as_quat()
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            # 将相机坐标系下的位姿转换为基座标系下的位姿
            base_pose = self.transPose(pose)
            #得到RPY角
            rpy = self.getRpy(euler_from_quaternion([base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w]))
            #从RPY角得到四元数
            quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            #将四元数赋值给base_pose
            base_pose.orientation.x = quaternion[0]
            base_pose.orientation.y = quaternion[1]
            base_pose.orientation.z = quaternion[2]
            base_pose.orientation.w = quaternion[3]

            pre_pose=Pose()
            pre_pose.position.x=base_pose.position.x
            pre_pose.position.y=base_pose.position.y
            pre_pose.position.z=base_pose.position.z+0.08
            #将四元数赋值给base_pose
            pre_pose.orientation.x = quaternion[0]
            pre_pose.orientation.y = quaternion[1]
            pre_pose.orientation.z = quaternion[2]
            pre_pose.orientation.w = quaternion[3]

            self.arm.set_pose_target(pre_pose, self.end_effector_link)
            
            # 规划路径
            plan_success, plan, planning_time, error_code = self.arm.plan()

            # 检查是否存在有效路径
            if not plan.joint_trajectory.points:
                rospy.logwarn("No valid path found. The goal pose may be unreachable, please planning the next goal pose")
                continue
            else:
                # 初始循环次数
                loop_count = 0

                while loop_count < 3:
                    waypoints = len(plan.joint_trajectory.points)
                    
                    # 如果waypoints < 40，则跳出循环并执行机械臂运动控制
                    if waypoints < 40:
                        rospy.loginfo("Waypoints数量小于40，继续执行")
                        break
                    else:
                        rospy.loginfo("Waypoints数量大于等于40，重新规划路径并继续循环判断")
                        # 重新规划路径
                        plan_success, plan, planning_time, error_code = self.arm.plan()
                        loop_count += 1

                # 如果循环次数达到10次仍未满足条件，则提示重新规划路径
                if loop_count == 3 and waypoints >= 40:
                    rospy.logwarn("重新规划路径超过3次，仍未满足条件，请检查目标位姿或路径规划算法")
                else:
                    # 控制机械臂完成运动
                    rospy.loginfo("enter键继续,q键退出")
                    key = input()
                    if key == 'q':
                        rospy.loginfo("已终止抓取，相机节点启动待命...")
                        return
                    else:
                        rospy.loginfo("继续执行")
                        # 控制机械臂完成运动
                        self.arm.execute(plan)

                        self.arm.set_pose_target(base_pose, self.end_effector_link)

                        # 规划路径
                        plan_success, plan, planning_time, error_code = self.arm.plan()

                        if not plan.joint_trajectory.points:
                            rospy.logwarn("No valid path found. The goal pose may be unreachable, please planning the next goal pose")
                            continue
                        # 存在有效路径，则继续执行
                        else:
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

                        #延迟1s
                        rospy.sleep(1)
                        # 完成夹取动作后，返回'home'姿态
                        self.move_to_home()

                        # 控制夹爪松开
                        self.gripper_control(0)
                        self.remove_object("plane1")
                        self.remove_object("plane2")
                        self.remove_object("plane3")
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
        #command = outputMsg.Robotiq2FGripper_robot_output()
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
        #rospy.sleep(0.1)

    def set_scene(self):
        ## set table
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        rospy.sleep(1)
        plane_id = 'plane1'
        self.scene.remove_world_object(plane_id)
        rospy.sleep(1)
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = "base_link"
        plane_pose.pose.position.x = 0.0
        plane_pose.pose.position.y = 0.0
        plane_pose.pose.position.z = 0.65
        plane_pose.pose.orientation.w = 1.0
        #self.scene.add_box(table_id, table_pose, table_size)
        self.scene.add_plane("plane1", plane_pose, normal=(0, 0, 1))
        plane_id2 = 'plane2'
        self.scene.remove_world_object(plane_id2)
        rospy.sleep(1)
        plane_pose2 = PoseStamped()
        plane_pose2.header.frame_id = "base_link"
        plane_pose2.pose.position.x = 0.0
        plane_pose2.pose.position.y = 0.0
        plane_pose2.pose.position.z = 0.0
        plane_pose2.pose.orientation.w = 1.0
        self.scene.add_plane("plane2", plane_pose2, normal=(0, 0, 1))
        plane_id3 = 'plane3'
        self.scene.remove_world_object(plane_id3)
        rospy.sleep(1)
        plane_pose3 = PoseStamped()
        plane_pose3.header.frame_id = "base_link"
        plane_pose3.pose.position.x = -0.2
        plane_pose3.pose.position.y = 0.0
        plane_pose3.pose.position.z = -0.5
        #rotation x=0 y=1.57 z=0
        quaternion = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        plane_pose3.pose.orientation.x = quaternion[0]
        plane_pose3.pose.orientation.y = quaternion[1]
        plane_pose3.pose.orientation.z = quaternion[2]
        plane_pose3.pose.orientation.w = quaternion[3]
        self.scene.add_plane("plane3", plane_pose3, normal=(0, 0, 1))

    #删除场景中的物体
    def remove_object(self, object_id):
        self.scene.remove_world_object(object_id)
        rospy.sleep(1)


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
                    self.moveit_aubo.set_scene()
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
