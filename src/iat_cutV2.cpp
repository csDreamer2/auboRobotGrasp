#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>

std::vector<std::string> commands;//ָ�
ros::Publisher iat_voice_cut;//������ɵ�����ָ����ⷢ��
bool is_last_command = false;// ����һ��ȫ�ֱ�������ʾ�Ƿ�Ϊ���һ��ָ��

#include <vector>
#include <string>
#include <algorithm>

#include <vector>
#include <string>
#include <algorithm>

std::vector<std::string> split(const std::string& text, const std::string& delim) {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;
    const std::string keywords[] = {"Ȼ��", "���", "����"};

    while ((end = text.find(delim, start)) != std::string::npos) {
        std::string token = text.substr(start, end - start);
        if (!token.empty()) {
            tokens.push_back(token);
        }
        start = end + delim.length();
    }

    std::string last_token = text.substr(start);
    if (!last_token.empty()) {
        tokens.push_back(last_token); // ������һ��Ƭ��
    }

    // ���ؼ����Ƿ������ÿ��Ƭ���У������������ж��β��
    std::vector<std::string> temp_tokens;
    for (const auto& token : tokens) {
        bool found = false;
        for (const auto& keyword : keywords) {
            size_t pos = token.find(keyword);
            if (pos != std::string::npos) {
                found = true;
                auto sub_tokens = split(token.substr(pos + keyword.length()), keyword);
                for (const auto& sub_token : sub_tokens) {
                    if (!sub_token.empty()) {
                        temp_tokens.push_back(sub_token);
                    }
                }
                if (!temp_tokens.empty()) {
                    temp_tokens.push_back(token.substr(0, pos));
                }
                break;
            }
        }
        if (!found) {
            temp_tokens.push_back(token);
        }
    }

    // �Ƴ����һ�����ַ���
    if (!temp_tokens.empty() && temp_tokens.back().empty()) {
        temp_tokens.pop_back();
    }

    return temp_tokens;
}

// ����ʶ��ص�����
void iattextCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string dataString = msg->data;
    
    commands = split(dataString, "��"); // ʹ���Զ����split����
    //std::cout << "Current commands:" << std::endl;
    for (size_t i = 0; i < commands.size(); ++i) {
        std_msgs::String command_msg;
        command_msg.data = commands[i];
        //std::cout << "split commands:" << commands[i] << std::endl;

        // �ж���Ϣ���Ƿ�����ؼ���
        if (commands[i].find("��ת") != std::string::npos ||
            commands[i].find("ץ") != std::string::npos ||
            commands[i].find("��ʼ��̬") != std::string::npos ||
            commands[i].find("�ټ�") != std::string::npos ||
            commands[i].find("ͬ��") != std::string::npos ||
            commands[i].find("��צ") != std::string::npos ||
            commands[i].find("�˳�") != std::string::npos){
            iat_voice_cut.publish(command_msg); // ������ iat_voice_cut ����
            std::cout  << "Current commands:" << commands[i] << std::endl;//����̨�������ָ��
        } else{
            // ���� chatgpt ����
            // Your logic for chatgpt processing here
            std::cout << "Calling chatgpt for: " << commands[i] << std::endl;
            continue;
        }
        // ��������һ��ָ���Ҳ��ȴ���������ѭ��
        if (i == commands.size() - 1 && !is_last_command) {
            break;
        }
        // ��ͣ3��
        ros::Duration(2).sleep();
    }
    std::cout << "������������������������" << std::endl;
}

//chatgpt�ش�ص�����
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string dataString = msg->data;
  std::cout << dataString.c_str() << std::endl;
}

int main(int argc, char **argv)
{
    // ��ʼ��ROS�ڵ�
    ros::init(argc, argv, "iat_voice_cut");

    // �����ڵ���
    ros::NodeHandle n;

     // ����һ�������ߣ�����String���͵���Ϣ����Ϊ'/iat_voice_cut'�Ļ���
    iat_voice_cut = n.advertise<std_msgs::String>("/iat_voice_cut", 1000);

    ros::Subscriber iat_text =n.subscribe("iat_text",1000,iattextCallback);

    //ros::Subscriber sub = n.subscribe("chatgpt", 1000, chatterCallback);


    ros::Rate loop_rate(10); // ����ѭ��Ƶ��Ϊ10Hz

    while (ros::ok()) {
        ros::spinOnce(); // ����ص�����
        loop_rate.sleep(); // ����ѭ��Ƶ������
    }

    exit:
    ros::shutdown();//�رսڵ�
    return 0;

}