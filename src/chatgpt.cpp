#include <ros/ros.h>
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"

class Robotiq2FGripper {
public:
    Robotiq2FGripper() {
        this->pub = this->nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    }

    void activate() {
        robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
        msg.rACT = 1;
        msg.rGTO = 1;
        msg.rSP = 255;
        msg.rFR = 150;
        this->publish(msg);
    }

    void reset() {
        robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
        msg.rACT = 0;
        this->publish(msg);
    }

    void setPosition(int position, int speed = 255, int force = 150) {
        if (position < 0 || position > 255) {
            ROS_WARN("Position must be between 0 and 255.");
            return;
        }
        robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
        msg.rACT = 1;
        msg.rGTO = 1;
        msg.rPR = position;
        msg.rSP = speed;
        msg.rFR = force;
        this->publish(msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    void publish(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_output& msg) {
        this->pub.publish(msg);
        ros::Duration(0.1).sleep(); // Give some time for the message to be sent
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotiq_2f_gripper_control_test");

    Robotiq2FGripper gripper;

    // Example usage
    gripper.reset();
    ros::Duration(2).sleep(); // Wait for the gripper to activate
    gripper.activate();
    ros::Duration(2).sleep(); // Wait for the gripper to move
    //gripper.setPosition(0); // Set gripper position
    return 0;
}