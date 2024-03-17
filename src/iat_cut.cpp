#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>

std::vector<std::string> commands; // ָ�
ros::Publisher iat_voice_cut; // ������ɵ�����ָ����ⷢ��

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

// �ж��Ƿ�����ؼ���
bool containsKeyword(const std::string& command) {
    std::vector<std::string> keywords = {"��ת", "ץ", "��ʼ��̬", "�ټ�", "ͬ��", "צ", "�˳�","�ſ�","�ر�","����"};
    for (const auto& keyword : keywords) {
        if (command.find(keyword) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// ����ʶ��ص�����
void iattextCallback(const std_msgs::String::ConstPtr& msg) {
    std::string dataString = msg->data;
    commands = split(dataString, "��");

    for (const auto& command : commands) {
        std_msgs::String command_msg;
        command_msg.data = command;

        if (containsKeyword(command)) {
            iat_voice_cut.publish(command_msg);
            std::cout << "Current command: " << command << std::endl;
        } else {
            // ���� chatgpt ����
            std::cout << "Calling chatgpt for: " << command << std::endl;
            continue;
        }
        // ��ͣ2��
        ros::Duration(2).sleep();
    }

    std::cout << "������������������������" << std::endl;
}

//chatgpt�ش�ص�����
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    std::string dataString = msg->data;
    std::cout << dataString << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "iat_voice_cut");
    ros::NodeHandle n;

    iat_voice_cut = n.advertise<std_msgs::String>("/iat_voice_cut", 1000);
    ros::Subscriber iat_text = n.subscribe("iat_text", 1000, iattextCallback);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}