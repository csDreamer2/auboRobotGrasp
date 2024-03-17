#include "ros/ros.h"
#include <std_msgs/Bool.h>  

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "example_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个发布者，发布String类型的消息到名为'/example_topic'的话题
    ros::Publisher pub = n.advertise<std_msgs::Bool>("/grasp", 10);
    
    // 创建一个String类型的消息
    std_msgs::Bool message;
    message.data = true;

    // 发布消息
    pub.publish(message);
    
    return 0;
}