import rospy
import tf2_ros
import geometry_msgs.msg

def transform_pose():
    rospy.init_node('transform_pose')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # 在这里定义您要转换的源坐标系和目标坐标系
            # 在这个例子中，源坐标系是机械臂相机的坐标系，目标坐标系是基座的坐标系
            trans = tfBuffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time())
            
            # 创建一个空的姿态消息
            pose_in_camera_frame = geometry_msgs.msg.PoseStamped()
            pose_in_camera_frame.header.frame_id = 'camera_color_optical_frame'
            pose_in_camera_frame.header.stamp = rospy.Time(0)

            # 在相机坐标系中填充位置信息（假设是在相机坐标系中的位置）
            pose_in_camera_frame.pose.position.x = 0.1
            pose_in_camera_frame.pose.position.y = 0.2
            pose_in_camera_frame.pose.position.z = 0.3
            
            # 在相机坐标系中填充朝向信息（假设是在相机坐标系中的朝向）
            pose_in_camera_frame.pose.orientation.x = 0.0
            pose_in_camera_frame.pose.orientation.y = 0.0
            pose_in_camera_frame.pose.orientation.z = 0.0
            pose_in_camera_frame.pose.orientation.w = 1.0

            # 进行坐标变换
            pose_transformed = tfBuffer.transform(pose_in_camera_frame, 'base_link')
            rospy.loginfo(pose_transformed)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        transform_pose()
    except rospy.ROSInterruptException:
        pass