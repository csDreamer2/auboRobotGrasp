#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "aubo_grasp/graspMessage.h"
#include <thread>

geometry_msgs::Twist twist;
ros::Publisher cmd_vel_pub;
std_msgs::String string_msg;//语音合成话题内容
ros::Publisher tts_text_pub;//语音合成话题发布者

aubo_grasp::graspMessage grasp_msg;// 抓取话题发布开关
ros::Publisher grasp_pub;//抓取话题发布者

int cmd_type = 0;//控制机械臂状况码
int joint_num = -1;//关节号
double angle4 = 0.0;//关节4角度
double angle3 = 90.0;//关节3角度
double rotation_angle = 0.0;//旋转角度
bool in_sync_mode = false;//同步模式标志

//同步模式回调函数
void armcallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 2) {
        double angle1 = msg->data[0];
        //ROS_INFO("Received angle1: %f", angle1);
        //如果关节4的角度小于-40则设置为-40，如果大于90则设置为90
        if(angle1 < 50) angle4 = -40;
        else if(angle1 > 180) angle4 = 90;
        else angle4 = angle1 -90;
        //设置关节3的角度为180 - angle2 如果关节3的角度小于30则设置为30，如果大于115则设置为115
        double angle2 = msg->data[1];
        if(angle2 < 65) angle3 = 115;
        else if(angle2 > 150) angle3 = 30;
        else angle3 = 180 - angle2;
        //ROS_INFO("Received angle4: %f, angle3: %f", angle4, angle3);
    } else {
        ROS_WARN("Received incomplete data.");
    }
}

//中文数字映射
std::map<std::string, int> chineseNumberMap = {
    {"一", 1},{"腰", 1},{"要", 1},{"妖", 1}, {"邀", 1},{"二", 2}, {"三", 3}, {"四", 4}, {"似", 4}, {"式", 4},
    {"五", 5}, {"舞", 5}, {"六", 6}, {"七", 7}, {"八", 8}, {"九", 9}, {"十", 10},
    {"十一", 11}, {"十二", 12}, {"十三", 13}, {"十四", 14}, {"十五", 15}, {"十六", 16},
    {"十七", 17}, {"十八", 18}, {"十九", 19}, {"二十", 20}
};

//获取关节号
int getJointNumber(const std::string& str) {
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            //如果pair.first大于1小于6则返回-1，否则返回pair.second
            if (pair.second > 6 || pair.second < 1) return -1;// 未识别的关节号
            else return pair.second;
        }
    }
    return -1;  // 未识别的关节号
}

//获取旋转角度
double getRotationAngle(const std::string& str) {
    if (str.empty()) return 0.0;
    int angle = 0;
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            angle = pair.second;
            break;
        }
    }
    if (angle == 0) {
        try {
            return std::stod(str);
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse rotation angle: %s", e.what());
        }
    }
    return angle;
}

//语音识别回调函数
void iattextCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;

    std::string dataString = msg->data;
    //打印语音识别结果
    std::string joint_num_str;
    std::string rotation_angle_str;
    //查找关键点pos
    size_t pos = dataString.find("旋转");
    if(pos != dataString.npos)
    {
        //设置joint_num_str关节参数 rotation_angle_str旋转角度参数
        joint_num_str = dataString.substr(0, pos);
        rotation_angle_str = dataString.substr(pos + 4);

        //获取关节号和旋转角度
        joint_num = getJointNumber(joint_num_str);
        rotation_angle = getRotationAngle(rotation_angle_str);
        if (joint_num != -1 && rotation_angle != 0.0) {
            ROS_INFO("Joint number: %d", joint_num);
            ROS_INFO("Rotation angle: %.1f", rotation_angle);
            //拼接语音
            char forwordString[40];
            snprintf(forwordString, sizeof(forwordString), "关节%d旋转%0.01f度", joint_num, rotation_angle);
            text = forwordString;
            //设置控制机械臂状况码
            cmd_type = 1;
        } else {
            ROS_WARN("Unrecognized joint number");
            text = "未识别到关节号";
        }
    }
    else if(dataString.find("初始姿态") != dataString.npos)
    {
        // Handle "初始姿态" command
        cmd_type = 2; // Assuming 2 is the command type for "零点姿态"
        text = "设置为初始姿态";
    }
    else if(dataString.find("直接抓取") != dataString.npos)
    {
        // Handle "抓取" command
        cmd_type = 3; // Assuming 3 is the command type for "抓取"
        text = "抓取";
    }
    else if(dataString.find("抓取盒子") != dataString.npos)
    {
        // Handle "抓取" command
        cmd_type = 7; // Assuming 3 is the command type for "抓取"
        text = "抓取盒子";
    }
    else if(dataString.find("抓取瓶") != dataString.npos)
    {
        // Handle "抓取" command
        cmd_type = 8; // Assuming 3 is the command type for "抓取"
        text = "抓取瓶子";
    }
    else if(dataString.find("抓取罐") != dataString.npos)
    {
        // Handle "抓取" command
        cmd_type = 9; // Assuming 3 is the command type for "抓取"
        text = "抓取罐子";
    }
    else if(dataString.find("抓取胶带") != dataString.npos)
    {
        // Handle "抓取" command
        cmd_type = 10; // Assuming 3 is the command type for "抓取"
        text = "抓取胶带";
    }
    //退出
    else if(dataString.find("再见") != dataString.npos)
    {
        // Handle "退出" command
        cmd_type = 4; // Assuming 4 is the command type for "退出"
        text = "已退出语音控制";
    }
    //同步模式
    else if(dataString.find("同步模式") != dataString.npos || dataString.find("同步姿态") != dataString.npos)
    {
        // Handle "同步" command
        //3秒后启动同步模式
        text = "5秒后启动同步模式";
        std::this_thread::sleep_for(std::chrono::seconds(5));
        cmd_type = 5; // Assuming 5 is the command type for "同步"
    }
    //退出同步
    else if(dataString.find("退出同步") != dataString.npos)
    {
         //Handle "退出同步" command
        in_sync_mode = false;
        cmd_type = 6; // Assuming 6 is the command type for "退出同步"
        text = "退出同步模式";
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
    ros::init(argc,argv,"e5_voice_control");
    ros::NodeHandle n;
    //开启异步spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //配置PLANNING_GROUP joint_model_group
    static const std::string PLANNING_GROUP = "manipulator_e5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //设置最大速度和加速度
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    //初始化同步模式、语音识别和语音合成
    ros::Subscriber arm_synchronization_sub = n.subscribe("arm_synchronization", 1000, armcallback);
    ros::Subscriber iat_text_sub =n.subscribe("iat_voice_cut", 1000,iattextCallback); //subscribe voice to text reault
    tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //初始化抓取话题
    //grasp_pub = n.advertise<std_msgs::Bool>("grasp", 10);
    grasp_pub = n.advertise<aubo_grasp::graspMessage>("grasp", 10);
    // 获取当前机械臂的状态
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    ROS_INFO("Wait Command...");
    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(!in_sync_mode){
        switch (cmd_type)
        {
        case 1:
            // 修改关节值 (将关节x旋转y度)
            joint_group_positions[joint_num - 1] += rotation_angle * M_PI / 180.0; // radians
            move_group.setJointValueTarget(joint_group_positions);
            // 规划并执行运动
            move_group.move();
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
            //message.data = true;
            grasp_msg.id = 1;
            grasp_msg.flag = true;
            grasp_msg.type = "hand";
            grasp_pub.publish(grasp_msg);
            cmd_type = 0;
            break;
        case 4:
            //退出
            goto exit;
            break;
        case 5:
            //同步模式
            in_sync_mode = true;
            break;
        case 6:
            //退出同步模式
            in_sync_mode = false;
            cmd_type = 2;
            break;
        case 7:
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "box";
            grasp_pub.publish(grasp_msg);
            cmd_type = 0;
            break;
        case 8:
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "bottle";
            grasp_pub.publish(grasp_msg);
            cmd_type = 0;
            break;
        case 9:
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "can";
            grasp_pub.publish(grasp_msg);
            cmd_type = 0;
            break;
        case 10:
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "sellotape";
            grasp_pub.publish(grasp_msg);
            cmd_type = 0;
            break;
        default:
            break;
        }
        }else{
            //同步模式
            //设置关节4的角度为angle4
            joint_group_positions[3] = angle4 * M_PI / 180.0;
            //设置关节3的角度为angle3
            joint_group_positions[2] = angle3 * M_PI / 180.0;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
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