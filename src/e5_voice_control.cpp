#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "aubo_grasp/graspMessage.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include <thread>

geometry_msgs::Twist twist;
ros::Publisher cmd_vel_pub;
std_msgs::String string_msg;//
ros::Publisher tts_text_pub;

aubo_grasp::graspMessage grasp_msg;// ץȡ������Ϣ������
ros::Publisher grasp_pub;//ץȡ���ⷢ����

int cmd_type = 0;//���ƻ�е��״̬
int joint_num = -1;//�ؽڱ��
double angle4 = 0.0;//�ؽ�4�Ƕ�
double angle3 = 90.0;//�ؽ�3�Ƕ�
double rotation_angle = 0.0;//��ת�Ƕ�
bool in_sync_mode = false;//ͬ��ģʽ��־

//ͬ��ģʽ�ص�����
void armcallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 2) {
        double angle1 = msg->data[0];
        //ROS_INFO("Received angle1: %f", angle1);
        //����ؽ�4�ĽǶ�С��-40������Ϊ-40���������90������Ϊ90
        if(angle1 < 50) angle4 = -40;
        else if(angle1 > 180) angle4 = 90;
        else angle4 = angle1 -90;
        //���ùؽ�3�ĽǶ�Ϊ180 - angle2 ����ؽ�3�ĽǶ�С��30������Ϊ30���������115������Ϊ115
        double angle2 = msg->data[1];
        if(angle2 < 65) angle3 = 115;
        else if(angle2 > 150) angle3 = 30;
        else angle3 = 180 - angle2;
        //ROS_INFO("Received angle4: %f, angle3: %f", angle4, angle3);
    } else {
        ROS_WARN("Received incomplete data.");
    }
}

//����������ӳ��Ϊ����
std::map<std::string, int> chineseNumberMap = {
    {"һ", 1},{"Ҫ", 1},{"��", 1},{"��", 1},{"��", 2}, {"��", 3}, {"��", 4}, {"��", 4}, {"ʽ", 4},
    {"��", 5}, {"��", 5}, {"��", 6}, {"��", 7}, {"��", 8}, {"��", 9}, {"ʮ", 10},
    {"ʮһ", 11}, {"ʮ��", 12}, {"ʮ��", 13}, {"ʮ��", 14}, {"ʮ��", 15}, {"ʮ��", 16},
    {"ʮ��", 17}, {"ʮ��", 18}, {"ʮ��", 19}, {"��ʮ", 20}
};

//��ȡ�ؽڱ��
int getJointNumber(const std::string& str) {
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            //���pair.first����1С��6�򷵻�-1�����򷵻�pair.second
            if (pair.second > 6 || pair.second < 1) return -1; // ʶ��Ĺؽں�
            else return pair.second;
        }
    }
    return -1;  // ʶ��Ĺؽں�
}

//��ȡ��ת�Ƕ�
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

//����ʶ��ص�����
void iattextCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;

    std::string dataString = msg->data;
    //��ӡ����ʶ����
    std::string joint_num_str;
    std::string rotation_angle_str;
    //���ҹؼ���pos
    size_t pos = dataString.find("��ת");
    if(pos != dataString.npos)
    {
        //����joint_num_str�ؽڲ��� rotation_angle_str��ת�ǶȲ���
        joint_num_str = dataString.substr(0, pos);
        rotation_angle_str = dataString.substr(pos + 4);

        //��ȡ�ؽںź���ת�Ƕ�
        joint_num = getJointNumber(joint_num_str);
        rotation_angle = getRotationAngle(rotation_angle_str);
        if (joint_num != -1 && rotation_angle != 0.0) {
            ROS_INFO("Joint number: %d", joint_num);
            ROS_INFO("Rotation angle: %.1f", rotation_angle);
            //ƴ������
            char forwordString[40];
            snprintf(forwordString, sizeof(forwordString), "�ؽ�%d��ת%0.01f��", joint_num, rotation_angle);
            text = forwordString;
            //���ÿ��ƻ�е��״̬
            cmd_type = 1;
        } else {
            ROS_WARN("Unrecognized joint number");
            text = "δʶ�𵽹ؽں�";
        }
    }
    else if(dataString.find("��ʼ��̬") != dataString.npos)
    {
        // Handle "��ʼ��̬" command
        cmd_type = 2; // Assuming 2 is the command type for "�����̬"
        text = "����Ϊ��ʼ��̬";
    }
    else if(dataString.find("ֱ��ץȡ") != dataString.npos)
    {
        // Handle "ץȡ" command
        cmd_type = 3; // Assuming 3 is the command type for "ץȡ"
        text = "ץȡ";
    }
    else if(dataString.find("ץȡ����") != dataString.npos || 
    dataString.find("ץ�����") != dataString.npos)
    {
        // Handle "ץȡ" command
        cmd_type = 7; // Assuming 3 is the command type for "ץȡ"
        text = "ץȡ����";
    }
    else if(dataString.find("ץȡƿ") != dataString.npos || 
    dataString.find("ץ��ƿ") != dataString.npos)
    {
        // Handle "ץȡ" command
        cmd_type = 8; // Assuming 3 is the command type for "ץȡ"
        text = "ץȡƿ��";
    }
    else if(dataString.find("ץȡ��") != dataString.npos || 
    dataString.find("ץ���") != dataString.npos)
    {
        // Handle "ץȡ" command
        cmd_type = 9; // Assuming 3 is the command type for "ץȡ"
        text = "ץȡ����";
    }
    else if(dataString.find("ץȡ����") != dataString.npos || 
    dataString.find("ץ�𽺴�") != dataString.npos)
    {
        // Handle "ץȡ" command
        cmd_type = 10; // Assuming 3 is the command type for "ץȡ"
        text = "ץȡ����";
    }
    //�˳�
    else if(dataString.find("�ټ�") != dataString.npos)
    {
        // Handle "�˳�" command
        cmd_type = 4; // Assuming 4 is the command type for "�˳�"
        text = "���˳���������";
    }
    //ͬ��ģʽ
    else if(dataString.find("ͬ��ģʽ") != dataString.npos || dataString.find("ͬ����̬") != dataString.npos)
    {
        // Handle "ͬ��ģʽ" command
        //3���ʼ����ͬ��ģʽ
        text = "3������ͬ��ģʽ";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        cmd_type = 5; // Assuming 5 is the command type for "ͬ��ģʽ"
    }
    //�˳�ͬ��ģʽ
    else if(dataString.find("�˳�ͬ��") != dataString.npos)
    {
         //Handle "�˳�ͬ��" command
        in_sync_mode = false;
        cmd_type = 6; // Assuming 6 is the command type for "�˳�ͬ��"
        text = "�˳�ͬ��ģʽ";
    }
    //��צ����
    else if(dataString.find("�ر�צ") != dataString.npos)
    {
        // Handle "��צ" command
        cmd_type = 11; // Assuming 3 is the command type for "ץȡ"
        text = "�պ�צ��";
    }
    else if(dataString.find("�ſ�צ") != dataString.npos)
    {
        // Handle "��צ" command
        cmd_type = 12; // Assuming 3 is the command type for "ץȡ"
        text = "�ſ�צ��";
    }
    else if(dataString.find("����צ") != dataString.npos)
    {
        // Handle "��צ" command
        cmd_type = 13; // Assuming 3 is the command type for "ץȡ"
        text = "�����צ";
    }
    else
    {
        ROS_WARN("unrecognized command");
        text = msg->data.c_str();
    }
    //�����ϳ�
    std::cout << text << std::endl;
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
}

//��צ���ƺ���������width
void gripperControl(double width)
{
    // ����һ��������
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // ����һ����צ��������
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
    // ���ü�צ
    command.rACT = 0;
    gripper_pub.publish(command);
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
    // �����צ
    command.rACT = 1;
    command.rGTO = 1;
    command.rSP = 255;
    command.rFR = 150;
    gripper_pub.publish(command);
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
    // ���ü�צ�Ŀ��
    command.rPR = static_cast<int>(width * 255);
    gripper_pub.publish(command);

    // �ȴ�����ȷ���������
    ros::Duration(0.1).sleep();
}

//�����צ����
void activeGripper()
{
    // ����һ��������
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // ����һ����צ��������
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
    // ���ü�צ
    command.rACT = 0;
    gripper_pub.publish(command);
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
    // �����צ
    command.rACT = 1;
    command.rGTO = 1;
    command.rSP = 255;
    command.rFR = 150;
    gripper_pub.publish(command);
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
}

int main(int argc, char* argv[])
{
    //��ʼ��ROS�ڵ�
    ros::init(argc,argv,"e5_voice_control");
    ros::NodeHandle n;
    //�����첽��Ϣ����
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //����PLANNING_GROUP joint_model_group
    static const std::string PLANNING_GROUP = "manipulator_e5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //��������ٶȺͼ��ٶ�
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    //��ʼ��ͬ��ģʽ������ʶ��������ϳ�
    ros::Subscriber arm_synchronization_sub = n.subscribe("arm_synchronization", 1000, armcallback);
    ros::Subscriber iat_text_sub =n.subscribe("iat_voice_cut", 1000,iattextCallback); //subscribe voice to text reault
    tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //��ʼ��ץȡ����
    //grasp_pub = n.advertise<std_msgs::Bool>("grasp", 10);
    grasp_pub = n.advertise<aubo_grasp::graspMessage>("grasp", 10);
    // ��ȡ��ǰ��е�۵�״̬
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
            // �ı�ؽڽǶ� (���ؽ�x��תy��)
            joint_group_positions[joint_num - 1] += rotation_angle * M_PI / 180.0; // radians
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = 0;
            break;
        case 2:
            //���û�е��homeλ��
            joint_group_positions[0] = 0;
            joint_group_positions[1] = 0;
            joint_group_positions[2] = 90 * M_PI / 180.0;
            joint_group_positions[3] = 0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = 0;
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = 0;
            break;
        case 3:
            //���û�е��ץȡ ����ץȡ���� message.data = true
            //message.data = true;
            grasp_msg.id = 1;
            grasp_msg.flag = true;
            grasp_msg.type = "hand";
            grasp_pub.publish(grasp_msg);
            cmd_type = 0;
            break;
        case 4:
            //�˳�
            goto exit;
            break;
        case 5:
            //ͬ��ģʽ
            in_sync_mode = true;
            break;
        case 6:
            //�˳�ͬ��ģʽ
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
        case 11:
            //�պ�צ��
            gripperControl(1);
            cmd_type = 0;
            break;
        case 12:
            //��צ��
            gripperControl(0);
            cmd_type = 0;
            break;
        case 13:
            //�����צ
            activeGripper();
            cmd_type = 0;
            break;
        default:
            break;
        }
        }else{
            //ͬ��ģʽ
            //���ùؽ�4�ĽǶ�Ϊangle4
            joint_group_positions[3] = angle4 * M_PI / 180.0;
            //���ùؽ�3�ĽǶ�Ϊangle3
            joint_group_positions[2] = angle3 * M_PI / 180.0;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
        }

        cmd_vel_pub.publish(twist);
        //ros::spinOnce();
        loop_rate.sleep();
    }

exit:
    spinner.stop();  // ���ʱֹͣ spinner
    ros::shutdown();
    return 0;
}