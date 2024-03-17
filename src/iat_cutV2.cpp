#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>

std::vector<std::string> commands;//指令集
ros::Publisher iat_voice_cut;//处理完成的语音指令集话题发布
bool is_last_command = false;// 定义一个全局变量来表示是否为最后一个指令

#include <vector>
#include <string>
#include <algorithm>

#include <vector>
#include <string>
#include <algorithm>

std::vector<std::string> split(const std::string& text, const std::string& delim) {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;
    const std::string keywords[] = {"然后", "最后", "接着"};

    while ((end = text.find(delim, start)) != std::string::npos) {
        std::string token = text.substr(start, end - start);
        if (!token.empty()) {
            tokens.push_back(token);
        }
        start = end + delim.length();
    }

    std::string last_token = text.substr(start);
    if (!last_token.empty()) {
        tokens.push_back(last_token); // 添加最后一个片段
    }

    // 检查关键词是否存在于每个片段中，如果存在则进行二次拆分
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

    // 移除最后一个空字符串
    if (!temp_tokens.empty() && temp_tokens.back().empty()) {
        temp_tokens.pop_back();
    }

    return temp_tokens;
}

// 语音识别回调函数
void iattextCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string dataString = msg->data;
    
    commands = split(dataString, "，"); // 使用自定义的split函数
    //std::cout << "Current commands:" << std::endl;
    for (size_t i = 0; i < commands.size(); ++i) {
        std_msgs::String command_msg;
        command_msg.data = commands[i];
        //std::cout << "split commands:" << commands[i] << std::endl;

        // 判断消息中是否包含关键词
        if (commands[i].find("旋转") != std::string::npos ||
            commands[i].find("抓") != std::string::npos ||
            commands[i].find("初始姿态") != std::string::npos ||
            commands[i].find("再见") != std::string::npos ||
            commands[i].find("同步") != std::string::npos ||
            commands[i].find("夹爪") != std::string::npos ||
            commands[i].find("退出") != std::string::npos){
            iat_voice_cut.publish(command_msg); // 发布到 iat_voice_cut 话题
            std::cout  << "Current commands:" << commands[i] << std::endl;//控制台输出单个指令
        } else{
            // 调用 chatgpt 处理
            // Your logic for chatgpt processing here
            std::cout << "Calling chatgpt for: " << commands[i] << std::endl;
            continue;
        }
        // 如果是最后一个指令且不等待，则跳出循环
        if (i == commands.size() - 1 && !is_last_command) {
            break;
        }
        // 暂停3秒
        ros::Duration(2).sleep();
    }
    std::cout << "————————————" << std::endl;
}

//chatgpt回答回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string dataString = msg->data;
  std::cout << dataString.c_str() << std::endl;
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "iat_voice_cut");

    // 创建节点句柄
    ros::NodeHandle n;

     // 创建一个发布者，发布String类型的消息到名为'/iat_voice_cut'的话题
    iat_voice_cut = n.advertise<std_msgs::String>("/iat_voice_cut", 1000);

    ros::Subscriber iat_text =n.subscribe("iat_text",1000,iattextCallback);

    //ros::Subscriber sub = n.subscribe("chatgpt", 1000, chatterCallback);


    ros::Rate loop_rate(10); // 设置循环频率为10Hz

    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        loop_rate.sleep(); // 按照循环频率休眠
    }

    exit:
    ros::shutdown();//关闭节点
    return 0;

}