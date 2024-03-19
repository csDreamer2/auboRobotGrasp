#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <map>
#include <functional>
#include <thread>
#include <atomic>
#include "geometry_msgs/Twist.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "aubo_grasp/graspMessage.h"
#include "aubo_grasp/followingMessage.h"

//ע�⣺����ʶ��������ϳɵ����ı���Ϊgb2312
geometry_msgs::Twist twist;//���ӻ��ڵ㻰������
ros::Publisher cmd_vel_pub;//���ӻ��ڵ㻰�ⷢ����
std_msgs::String string_msg;//�����ϳɻ�������
ros::Publisher tts_text_pub;//�����ϳɻ��ⷢ����

aubo_grasp::graspMessage grasp_msg;// ץȡ���ⷢ����Ϣ
ros::Publisher grasp_pub;//ץȡ���ⷢ����
aubo_grasp::followingMessage following_msg;// ������Ʒ���ⷢ����Ϣ
ros::Publisher following_pub;//������Ʒ���ⷢ����

//����ȫ�ֱ���
int cmd_type = -1;//���ƻ�е��״����
int joint_num = -1;//�ؽں�
double angle3 = 90.0;//�ؽ�3�Ƕ�
double angle4 = 0.0;//�ؽ�4�Ƕ�
double gripper_width = 0.0;//��צ���
double rotation_angle = 0.0;//��ת�Ƕ�
bool in_sync_mode = false;//ͬ��ģʽ��־
std::atomic<bool> gripperThreadRunning(true); // ����һ��ԭ�Ӳ���ֵ����ʾ��צ�����߳��Ƿ�����

// ������������
enum CommandType {
    EXIT = 0,//�˳�
    JOINT_ROTATE = 1,//�ؽ���ת
    INITIAL_POSE = 2,//��ʼ��̬
    DIRECT_GRASP = 3,//ֱ��ץȡ
    GRASP_HAND = 4,//ץ��
    GRASP_BOX = 7,//ץȡ����
    GRASP_BOTTLE = 8,//ץȡƿ
    GRASP_CAN = 9,//ץȡ��
    GRASP_TAPE = 10,//ץȡ����
    SYNC_MODE = 5,//ͬ��ģʽ
    EXIT_SYNC = 6,//�˳�ͬ��
    GRIPPER_CLOSE = 11,//��צ�պ�
    GRIPPER_OPEN = 12,//��צ��
    GRIPPER_ACTIVE = 13,//�����צ
    FOLLOWING_BOTTLE = 14,//����ƿ��
    FOLLOWING_BOX = 15,//�������
    FOLLOWING_CAN = 16,//�������
    FOLLOWING_TAPE = 17,//���潺��
};

// ���崦��������
typedef std::function<void(const std::string&)> CommandHandler;
void handleExit(const std::string& dataString);
void handleJointRotate(const std::string& dataString);
void handleInitialPose(const std::string& dataString);
void handleDirectGrasp(const std::string& dataString);
void handleGraspHand(const std::string& dataString);
void handleGraspBox(const std::string& dataString);
void handleGraspBottle(const std::string& dataString);
void handleGraspCan(const std::string& dataString);
void handleGraspTape(const std::string& dataString);
void handleSyncMode(const std::string& dataString);
void handleExitSync(const std::string& dataString);
void handleGripperClose(const std::string& dataString);
void handleGripperOpen(const std::string& dataString);
void handleGripperActive(const std::string& dataString);
void handleFollowingBottle(const std::string& dataString);
void handleFollowingBox(const std::string& dataString);
void handleFollowingCan(const std::string& dataString);
void handleFollowingTape(const std::string& dataString);

// �洢�ؼ��ʺͶ�Ӧ�Ĵ������Լ���������
std::map<std::string, std::pair<CommandHandler, CommandType>> commandMap = {
    {"�ټ�", {handleExit, EXIT}},
    {"��ת", {handleJointRotate, JOINT_ROTATE}},
    {"��ʼ��̬", {handleInitialPose, INITIAL_POSE}},
    {"ֱ��ץȡ", {handleDirectGrasp, DIRECT_GRASP}},
    {"ץ��", {handleDirectGrasp, GRASP_HAND}},
    {"ץȡ����", {handleGraspBox, GRASP_BOX}},
    {"ץ�����", {handleGraspBox, GRASP_BOX}},
    {"ץȡƿ", {handleGraspBottle, GRASP_BOTTLE}},
    {"ץ��ƿ", {handleGraspBottle, GRASP_BOTTLE}},
    {"ץȡ��", {handleGraspCan, GRASP_CAN}},
    {"ץ���", {handleGraspCan, GRASP_CAN}},
    {"ץȡ����", {handleGraspTape, GRASP_TAPE}},
    {"ץ�𽺴�", {handleGraspTape, GRASP_TAPE}},
    {"ͬ��ģʽ", {handleSyncMode, SYNC_MODE}},
    {"ͬ����̬", {handleSyncMode, SYNC_MODE}},
    {"�˳�ͬ��", {handleExitSync, EXIT_SYNC}},
    {"�ر�צ", {handleGripperClose, GRIPPER_CLOSE}},
    {"�պϼ�צ", {handleGripperClose, GRIPPER_CLOSE}},
    {"�ſ�צ", {handleGripperOpen, GRIPPER_OPEN}},
    {"�ſ�ת", {handleGripperOpen, GRIPPER_OPEN}},
    {"�ſ���צ", {handleGripperOpen, GRIPPER_OPEN}},
    {"�����צ", {handleGripperActive, GRIPPER_ACTIVE}},
    {"�����״", {handleGripperActive, GRIPPER_ACTIVE}},
    {"���צ", {handleGripperActive, GRIPPER_ACTIVE}},
    {"����ƿ", {handleFollowingBottle, FOLLOWING_BOTTLE}},
    {"�����", {handleFollowingBox, FOLLOWING_BOX}},
    {"�����", {handleFollowingCan, FOLLOWING_CAN}},
    {"���潺��", {handleFollowingTape, FOLLOWING_TAPE}},
};

//��������ӳ��
std::map<std::string, int> chineseNumberMap = {
    {"һ", 1},{"��", 1},{"Ҫ", 1},{"��", 1}, {"��", 1},{"��", 2}, {"��", 3}, {"��", 4}, {"��", 4}, {"ʽ", 4},
    {"��", 5}, {"��", 5}, {"��", 6}, {"��", 7}, {"��", 8}, {"��", 9}, {"ʮ", 10},
    {"ʮһ", 11}, {"ʮ��", 12}, {"ʮ��", 13}, {"ʮ��", 14}, {"ʮ��", 15}, {"ʮ��", 16},
    {"ʮ��", 17}, {"ʮ��", 18}, {"ʮ��", 19}, {"��ʮ", 20}
};

//ͬ��ģʽ�ص�����
void armcallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 4) {
        //��ӡ4������
        //ROS_INFO("Received data: %f", msg->data[2]);
        double angle1 = msg->data[0];
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
        //���thumb_index_angleС��4������Ϊ4���������45������Ϊ45
        double thumb_index_data = msg->data[2];
        if(thumb_index_data < 6) gripper_width = 1.0;
        else if(thumb_index_data > 45) gripper_width = 0.0;
        else gripper_width = 1.0 - (thumb_index_data - 6) / 39.0;
        //ROS_INFO("Received data: %f", gripper_width);
    } else {
        ROS_WARN("Received incomplete data.");
    }
}

// ����ʶ��ص�����
void iattextCallback(const std_msgs::String::ConstPtr& msg) {
    const char* text;
    std::string dataString = msg->data;
    //ָ��ʶ��
    for (const auto& pair : commandMap) {
        if (dataString.find(pair.first) != std::string::npos) {
            pair.second.first(dataString);
            //ROS_INFO("Received command: %s", pair.first.c_str());
            cmd_type = pair.second.second;
            return;
        }
    }
    //δʶ������������ϳɷ���
    ROS_WARN("Unrecognized command");
    cmd_type = -1;
    text = "δʶ�������";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
}

//��ȡ�ؽں�
int getJointNumber(const std::string& str) {
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            //���pair.first����1С��6�򷵻�-1�����򷵻�pair.second
            if (pair.second > 6 || pair.second < 1) return -1;// δʶ��Ĺؽں�
            else return pair.second;
        }
    }
    return -1;  // δʶ��Ĺؽں�
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

// �����˳�����
void handleExit(const std::string& dataString) {
    const char* text;
    text = "���˳���������";
}

// ����ؽ���ת����
void handleJointRotate(const std::string& dataString) {
    const char* text;
    //��ӡ����ʶ����
    std::string joint_num_str;
    std::string rotation_angle_str;
    size_t pos = dataString.find("��ת");
    //����joint_num_str�ؽڲ��� rotation_angle_str��ת�ǶȲ���
    joint_num_str = dataString.substr(0, pos);
    rotation_angle_str = dataString.substr(pos + 4);//gb2312�����ʽ��"��ת"�ĳ���Ϊ4��utf8Ϊ6��
    //��ȡ�ؽںź���ת�Ƕ�
    joint_num = getJointNumber(joint_num_str);
    rotation_angle = getRotationAngle(rotation_angle_str);
    if (joint_num != -1 && rotation_angle != 0.0) {
        //����̨����ؽ�%d��ת%.1f��
        ROS_INFO("Received command: joint %d rotate %.1f degrees", joint_num, rotation_angle);
        //ƴ�����������������ϳɻ���
        char forwordString[40];
        snprintf(forwordString, sizeof(forwordString), "�ؽ�%d��ת%.1f��", joint_num, rotation_angle);
        text = forwordString;
        string_msg.data = text;
        tts_text_pub.publish(string_msg);
    } else {
        ROS_WARN("Unrecognized joint number or rotation angle");
        text = "δʶ�𵽹ؽںŻ���ת�Ƕ�";  
        string_msg.data = text;
        tts_text_pub.publish(string_msg);      
    }
}

// �����ʼ��̬����
void handleInitialPose(const std::string& dataString) {
    const char* text;
    text = "����Ϊ��ʼ��̬";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to initial pose");
}

// ����ֱ��ץȡ����
void handleDirectGrasp(const std::string& dataString) {
    const char* text;
    text = "��ʼֱ��ץȡ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: direct grasp");
}

// ����ץ������
void handleGraspHand(const std::string& dataString) {
    const char* text;
    text = "��ʼץ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp hand");
}

// ����ץȡ��������
void handleGraspBox(const std::string& dataString) {
    const char* text;
    text = "��ʼץȡ����";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp box");
}

// ����ץȡƿ������
void handleGraspBottle(const std::string& dataString) {
    const char* text;
    text = "��ʼץȡƿ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp bottle");
}

// ����ץȡ��������
void handleGraspCan(const std::string& dataString) {
    const char* text;
    text = "��ʼץȡ����";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp can");
}

// ����ץȡ��������
void handleGraspTape(const std::string& dataString) {
    const char* text;
    text = "��ʼץȡ����";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp tape");
}

// ����ͬ��ģʽ����
void handleSyncMode(const std::string& dataString) {
    const char* text;
    in_sync_mode = true;
    text = "3�������ͬ��ģʽ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: after 3s start sync mode");
    std::this_thread::sleep_for(std::chrono::seconds(3));//3�������ͬ��ģʽ
    text = "��ʼͬ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: start sync mode");
}

// �����˳�ͬ������
void handleExitSync(const std::string& dataString) {
    const char* text;
    in_sync_mode = false;
    text = "�˳�ͬ��ģʽ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: exit sync mode");
}

// �����צ�պ�����
void handleGripperClose(const std::string& dataString) {
    const char* text;
    text = "��צ�պ�";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper close");
}

// �����צ������
void handleGripperOpen(const std::string& dataString) {
    const char* text;
    text = "��צ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper open");
}

// �������צ����
void handleGripperActive(const std::string& dataString) {
    const char* text;
    text = "�����צ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper active");
}

// �������ƿ������
void handleFollowingBottle(const std::string& dataString) {
    const char* text;
    text = "��ʼ����ƿ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: following bottle");
}

// ��������������
void handleFollowingBox(const std::string& dataString) {
    const char* text;
    text = "��ʼ�������";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: following box");
}

// ��������������
void handleFollowingCan(const std::string& dataString) {
    const char* text;
    text = "��ʼ�������";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: following can");
}

// ������潺������
void handleFollowingTape(const std::string& dataString) {
    const char* text;
    text = "��ʼ���潺��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: following tape");
}

//��צ���ƺ���������width
void gripperControl(double width)
{
    // ����һ��������
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // ����һ����צ��������
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
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
    ROS_INFO("Gripper width set to %.2f", width);
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
     ros::Duration(1.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
    // �����צ
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output activeGripperCommand;
    activeGripperCommand.rACT = 1;
    activeGripperCommand.rGTO = 1;
    activeGripperCommand.rSP = 255;
    activeGripperCommand.rFR = 150;
    gripper_pub.publish(activeGripperCommand);
    ROS_INFO("Gripper activated");
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
}

// �̼߳�צ���ƺ���
void gripperControlThread(double width) {
    // ����צ�����̱߳�־Ϊtrueʱ��ÿ��2000msִ��һ�μ�צ����
    while (gripperThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        gripperControl(width);
    }
    // �˳��߳�
    ROS_INFO("Gripper control thread exited");
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
    //��ʼ��ͬ��ģʽ������ʶ��������ϳɡ����ӻ��ڵ㻰��
    ros::Subscriber arm_synchronization_sub = n.subscribe("arm_synchronization", 1000, armcallback);
    ros::Subscriber iat_text_sub =n.subscribe("iat_voice_cut", 1000,iattextCallback); //subscribe voice to text reault
    tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);//���ӻ��ڵ㻰�ⷢ��
    //��ʼ��ץȡ����
    grasp_pub = n.advertise<aubo_grasp::graspMessage>("grasp", 10);
    //��ʼ�����滰��
    following_pub = n.advertise<aubo_grasp::followingMessage>("followingObject", 10);
    //��ȡ��ǰ��е�۵�״̬
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //��ʼ����צ
    activeGripper();
    //�ȴ���е�۳�ʼ��
    ROS_INFO("Wait Command...");
    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(!in_sync_mode){
        switch (cmd_type)
        {
        case 0:
            //�˳�
            goto exit;
            break;
        case 1:
            // �ı�ؽڽǶ� (���ؽ�x��תy��)
            joint_group_positions[joint_num - 1] += rotation_angle * M_PI / 180.0; // radians
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = -1;
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
            cmd_type = -1;
            break;
        case 3:
            //���û�е��ֱ��ץȡ ����ץȡ���� id=1
            grasp_msg.id = 1;
            grasp_msg.flag = true;
            grasp_msg.type = "";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 4:
            //���û�е��ץ�� ����ץȡ���� id=2 type=hand
            grasp_msg.id = 1;
            grasp_msg.flag = true;
            grasp_msg.type = "hand";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 5:
            //ͬ��ģʽ
            in_sync_mode = true;
            //���ü�צ�����̱߳�־Ϊtrue
            gripperThreadRunning = true;
            break;
        case 6:
            //�˳�ͬ��ģʽ
            in_sync_mode = false;
            //���ü�צ�����̱߳�־Ϊfalse��֪ͨ�߳�ִֹͣ��
            gripperThreadRunning = false;
            gripperControl(0);
            cmd_type = 2;
            break;
        case 7:
            //���û�е��ץȡ���� ����ץȡ���� id=2 type=box
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "box";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 8:
            //���û�е��ץȡƿ�� ����ץȡ���� id=2 type=bottle
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "bottle";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 9:
            //���û�е��ץȡ���� ����ץȡ���� id=2 type=can
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "can";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 10:
            //���û�е��ץȡ���� ����ץȡ���� id=2 type=sellotape
            grasp_msg.id = 2;
            grasp_msg.flag = true;
            grasp_msg.type = "sellotape";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 11:
            //�պϼ�צ
            gripperControl(1);
            cmd_type = -1;
            break;
        case 12:
            //�ſ���צ
            gripperControl(0);
            cmd_type = -1;
            break;
        case 13:
            //�����צ
            activeGripper();
            cmd_type = -1;
            break;
        case 14:
            //���û�е�۸���ƿ�� �������滰�� type=bottle
            following_msg.flag = true;
            following_msg.type = "bottle";
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        case 15:
            //���û�е�۸������ �������滰�� type=box
            following_msg.flag = true;
            following_msg.type = "box";
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        case 16:
            //���û�е�۸������ �������滰�� type=can
            following_msg.flag = true;
            following_msg.type = "can";
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        case 17:
            //���û�е�۸��潺�� �������滰�� type=sellotape
            following_msg.flag = true;
            following_msg.type = "sellotape";
            following_pub.publish(following_msg);
            cmd_type = -1;
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
            // ����һ�����߳�ִ�м�צ���ƣ�����������Ϊ��̨�߳�
            //std::thread gripperThread(gripperControlThread, gripper_width);
            //gripperThread.detach(); 
            //���ü�צ���
            gripperControl(gripper_width);
            //ִ�л�е���ƶ�
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