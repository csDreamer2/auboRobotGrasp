import openai
import rospy
from std_msgs.msg import String


# openai.log = "debug"
openai.api_key = "sk-lQqOOZhzhWxyGwxNJ2oAJBkshxe22FyCzp64bqjlPpK88OdS"
openai.api_base = "https://api.chatanywhere.tech/v1"

# 非流式响应
# completion = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=[{"role": "user", "content": "Hello world!"}])
# print(completion.choices[0].message.content)

def gpt_35_api_stream(messages: list):
    """为提供的对话消息创建新的回答 (流式传输)

    Args:
        messages (list): 完整的对话消息
        api_key (str): OpenAI API 密钥

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
                print(f'收到的完成数据: {completion}')
                break
            for delta_k, delta_v in event['choices'][0]['delta'].items():
                print(f'流响应数据: {delta_k} = {delta_v}')
                completion[delta_k] += delta_v
        messages.append(completion)  # 直接在传入参数 messages 中追加消息
        return (True, '')
    except Exception as err:
        return (False, f'OpenAI API 异常: {err}')
    
def publish_chat_topic():
    # 初始化ROS节点
    rospy.init_node('gptchat_publisher', anonymous=True)
    
    # 创建一个发布者，发布String类型的消息到名为'/chat'的话题
    chat_pub = rospy.Publisher('/chatgpt', String, queue_size=10)
    
    # 设置循环的频率，这里为1Hz
    rate = rospy.Rate(1) 

    # 循环发布消息
    while not rospy.is_shutdown():
        # 创建一个String类型的消息
        message = "Hello, world!"
        
        # 发布消息
        chat_pub.publish(message)
        
        # 打印发布的消息
        rospy.loginfo("Publishing: %s" % message)
        
        # 按照设定的频率休眠
        rate.sleep()


if __name__ == '__main__':
    messages = [{'role': 'user','content': '你好'},]
    print(gpt_35_api_stream(messages))#返回值为True
    #print(messages)
    try:
        publish_chat_topic()
    except rospy.ROSInterruptException:
        pass