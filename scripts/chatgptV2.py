import openai
import rospy
from std_msgs.msg import String

# 设置OpenAI API密钥和基础网址
openai.api_key = "sk-lQqOOZhzhWxyGwxNJ2oAJBkshxe22FyCzp64bqjlPpK88OdS"
openai.api_base = "https://api.chatanywhere.tech/v1"

def gpt_35_api_stream(messages: list):
    """为提供的对话消息创建新的回答 (流式传输)

    Args:
        messages (list): 完整的对话消息

    Returns:
        tuple: (results, error_desc)
    """
    try:
        response = openai.ChatCompletion.create(
            model='gpt-3.5-turbo',
            messages=messages,
            stream=True,
        )
        completion = {'role': '', 'content': ''}
        for event in response:
            if event['choices'][0]['finish_reason'] == 'stop':
                #print(f'收到的完成数据: {completion}')
                break
            for delta_k, delta_v in event['choices'][0]['delta'].items():
                #print(f'流响应数据: {delta_k} = {delta_v}')
                completion[delta_k] += delta_v
        messages.append(completion)  # 直接在传入参数 messages 中追加消息
        return (True, '')
    except Exception as err:
        return (False, f'OpenAI API 异常: {err}')
    
def chat_text_callback(data):
    print("Received data:", data.data)
    global messages
    message_content = data.data
    new_message = {'role': 'user', 'content': message_content}
    messages.append(new_message)

def publish_gpt_response():
    # 初始化ROS节点
    rospy.init_node('gptchat_publisher', anonymous=True)
    
    # 创建一个发布者，发布String类型的消息到名为'/chatgpt'的话题
    chatgpt_pub = rospy.Publisher('/chatgpt', String, queue_size=10)
    
    # 设置循环的频率，这里为1Hz
    rate = rospy.Rate(1) 

    # 循环发布消息
    while not rospy.is_shutdown():
        global messages
        if messages:
            #判断messsages中是否有“抓取”“初始姿态”“旋转”关键词，如果没有则进行发布，有则继续等待
            if "抓取" in messages[-1]['content'] or "初始姿态" in messages[-1]['content'] or "旋转" in messages[-1]['content']:
                #控制台打印消息
                rospy.loginfo("Not publish GPT response")
                pass
            else:
                # 调用OpenAI API获取回答
                success, error_desc = gpt_35_api_stream(messages)
                if success:
                    response_content = messages[-1]['content']  # 获取最新的回答内容
                    # 创建一个String类型的消息
                    response_msg = String()
                    response_msg.data = response_content
                    # 发布消息
                    chatgpt_pub.publish(response_msg)
                    # 打印发布的消息
                    rospy.loginfo("Published GPT response: %s" % response_content)
                    # 清空消息列表
                    messages = []
                else:
                    rospy.logerr("Failed to generate GPT response: %s" % error_desc)
        
        # 按照设定的频率休眠
        rate.sleep()


if __name__ == '__main__':
    messages = [{'role': 'user','content': '你好'},]
    
    # 订阅名为"iat_text"的ROS话题，以获取指令消息
    rospy.Subscriber("iat_text", String, chat_text_callback)
    
    try:
        publish_gpt_response()
    except rospy.ROSInterruptException:
        pass