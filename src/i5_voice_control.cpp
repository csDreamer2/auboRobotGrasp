#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist twist;
ros::Publisher cmd_vel_pub;
std_msgs::String string_msg;
ros::Publisher tts_text_pub;
std_msgs::Bool message;
ros::Publisher grasp_pub;

int cmd_type = 0;
int joint_num = 0;
double rotation_angle = 0.0;

void iattextCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;

    std::string dataString = msg->data;
    std::string joint_num_str;
    std::string rotation_angle_str;
    size_t pos = dataString.find("旋转");
    if(pos != dataString.npos)
    {
        //设置joint_num_str关节参数 rotation_angle_str旋转角度参数
        joint_num_str = dataString.substr(0, pos);
        rotation_angle_str = dataString.substr(pos + 6); // 6 is the length of "旋转"
        //1.对关节部分语音处理
        if(joint_num_str.find("一") != joint_num_str.npos) joint_num = 1;
        else if(joint_num_str.find("二") != joint_num_str.npos) joint_num = 2;
        else if(joint_num_str.find("三") != joint_num_str.npos) joint_num = 3;
        else if(joint_num_str.find("四") != joint_num_str.npos || joint_num_str.find("似") != joint_num_str.npos || joint_num_str.find("式") != joint_num_str.npos)
        {
            if(joint_num_str.find("六") != joint_num_str.npos) joint_num = 6;
            else joint_num = 4;
        }
        else if(joint_num_str.find("五") != joint_num_str.npos || joint_num_str.find("舞") != joint_num_str.npos)
        {
            if(joint_num_str.find("六") != joint_num_str.npos) joint_num = 6;
            else joint_num = 5;
        }
        else if(joint_num_str.find("六") != joint_num_str.npos) joint_num = 6;
        else
        {
        //删“关节“二字
        std::string afterSixthChar = joint_num_str.substr(6);
        joint_num = std::stoi(afterSixthChar);
        }
        //2.对旋转角度部分语音处理
        if(rotation_angle_str.find("一") != rotation_angle_str.npos) rotation_angle = 1;
        else if(rotation_angle_str.find("二") != rotation_angle_str.npos) rotation_angle = 2;
        else if(rotation_angle_str.find("三") != rotation_angle_str.npos) rotation_angle = 3;
        else if(rotation_angle_str.find("四") != rotation_angle_str.npos || rotation_angle_str.find("似") != rotation_angle_str.npos || rotation_angle_str.find("式") != rotation_angle_str.npos)
        {
            if(rotation_angle_str.find("六") != rotation_angle_str.npos) rotation_angle = 6;
            else rotation_angle = 4;
        }
        else if(rotation_angle_str.find("五") != rotation_angle_str.npos || rotation_angle_str.find("舞") != rotation_angle_str.npos)
        {
            if(rotation_angle_str.find("六") != rotation_angle_str.npos) rotation_angle = 6;
            else rotation_angle = 5;
        }
        else if(rotation_angle_str.find("六") != rotation_angle_str.npos) rotation_angle = 6;
        else{
        rotation_angle = std::stod(rotation_angle_str);
        }
        //拼接语音
        char forwordString[40];
        snprintf(forwordString, sizeof(forwordString), "关节%d旋转%0.01f度", joint_num, rotation_angle);
        text = forwordString;
        //设置控制机械臂状况码
        cmd_type = 1;
    }
    else if(dataString.find("初始姿态") != dataString.npos)
    {
        // Handle "初始姿态" command
        cmd_type = 2; // Assuming 2 is the command type for "零点姿态"
        text = "设置为初始姿态";
    }
    else if(dataString.find("抓取") != dataString.npos)
    {
        // Handle "抓取" command
        cmd_type = 3; // Assuming 3 is the command type for "抓取"
        text = "抓取";
    }
    //退出
    else if(dataString.find("再见") != dataString.npos)
    {
        // Handle "退出" command
        cmd_type = 4; // Assuming 4 is the command type for "退出"
        text = "已退出语音控制";
    }
    else
    {
        ROS_WARN("unrecognized command");
        text = msg->data.c_str();
    }
    //语音合成
    std::cout << text << std::endl;
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
}

int main(int argc, char* argv[])
{
    //初始化ROS节点
    ros::init(argc,argv,"i5_voice_control");
    ros::NodeHandle n;
    //开启异步spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //配置PLANNING_GROUP joint_model_group
    static const std::string PLANNING_GROUP = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //初始化语音识别和语音合成
    ros::Subscriber iat_text_sub =n.subscribe("iat_text", 1000,iattextCallback); //subscribe voice to text reault
    tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //初始化抓取话题
    grasp_pub = n.advertise<std_msgs::Bool>("grasp", 10);
    // 获取当前机械臂的状态
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    ROS_INFO("Wait Command...");
    ros::Rate loop_rate(50);
    while(ros::ok()){
        switch (cmd_type)
        {
        case 1:
            // 修改关节值 (将关节x旋转y度)
            //joint_group_positions[joint_num - 1] += rotation_angle * M_PI / 180.0; // radians
            //move_group.setJointValueTarget(joint_group_positions);
            ROS_INFO("123");
            // 规划并执行运动
            //move_group.move();
            cmd_type = 0;
            break;
        case 2:
            //设置机械臂home位置
            joint_group_positions[0] = 0;
            joint_group_positions[1] = 0;
            joint_group_positions[2] = 90 * M_PI / 180.0;
            joint_group_positions[3] = 0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = 0;
            move_group.setJointValueTarget(joint_group_positions);
            // 规划并执行运动
            move_group.move();
            cmd_type = 0;
            break;
        case 3:
            //设置机械臂抓取 发布抓取话题message.data = true
            message.data = true;
            grasp_pub.publish(message);
            cmd_type = 0;
            break;
        case 4:
            //退出
            goto exit;
            break;
        default:
            break;
        }
        cmd_vel_pub.publish(twist);
        //ros::spinOnce();
        loop_rate.sleep();
    }

exit:
    spinner.stop();  // 完成时停止 spinner
    ros::shutdown();
    return 0;
}