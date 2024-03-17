from tf import TransformListener
from geometry_msgs.msg import PoseStamped
import rospy

def main():
    
    rospy.init_node('tf_test')
    rospy.loginfo("Node for testing actionlib server")
 
    #from_link = '/head_color_camera_l_link'
    #to_link = '/base_link'
    
    from_link = '/camera_color_optical_frame'
    to_link = '/base_link'
    
    tfl = TransformListener()
    
    rospy.sleep(0.5)
    
    t = rospy.Time(0)
    
    mpose = PoseStamped()
    
    mpose.pose.position.x = 1
    mpose.pose.position.y = 0
    mpose.pose.position.z = 0
    
    mpose.pose.orientation.x = 0
    mpose.pose.orientation.y = 0
    mpose.pose.orientation.z = 0
    mpose.pose.orientation.w = 0
    
    mpose.header.frame_id = from_link
    mpose.header.stamp = rospy.Time.now()
    
    rospy.sleep(0.5)
    
    mpose_transf = None
    
    rospy.loginfo('Waiting for transform for some time...')
    
    tfl.waitForTransform(to_link,from_link,t,rospy.Duration(5))
    
    if tfl.canTransform(to_link,from_link,t):
      
      mpose_transf = tfl.transformPose(to_link,mpose)
      
      print (mpose_transf)
      
    else:
      
      rospy.logerr('Transformation is not possible!')
      exit(0)

if __name__ == '__main__':
    main()