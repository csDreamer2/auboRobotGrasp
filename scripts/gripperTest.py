import rospy
from graspnetapi import MoveItAubo


def test_gripper_control():
    rospy.init_node('test_gripper_control_node')
    rospy.loginfo("Initializing test gripper control node...")

    # 创建 MoveItAubo 实例
    moveit_aubo = MoveItAubo()

    # 设置夹爪宽度
    gripper_width = 0  # 这里设置为你想要的夹爪宽度

    # 调用 gripper_control 方法
    moveit_aubo.gripper_control(gripper_width)

    rospy.loginfo("Gripper control test complete.")

if __name__ == '__main__':
    try:
        test_gripper_control()
    except rospy.ROSInterruptException:
        pass