#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotate_joint1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator_i5";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Get the current joint values to start with
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  //打印当前机械臂的位置信息
  ROS_INFO_NAMED("tutorial", "Current joint positions: %f %f %f %f %f %f", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2], joint_group_positions[3], joint_group_positions[4], joint_group_positions[5]);

  // Modify the value of joint 1 (rotate 20 degrees)
  //joint_group_positions[0] += 20.0 * M_PI / 180.0;  // radians
  //将机械臂6个关节的位置信息分别设置为0° 0° 90° 0° 90° 0°
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 90 * M_PI / 180.0;
  joint_group_positions[3] = 0;
  joint_group_positions[4] = 90 * M_PI / 180.0;
  joint_group_positions[5] = 0;
  move_group.setJointValueTarget(joint_group_positions);
  // Call the planner for planning calculations Note: This is just planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

 //按enter键继续
  std::cin.ignore();

  if(success)
  {
    ROS_INFO_NAMED("tutorial", "Plan success, executing...");
    move_group.execute(my_plan);
  }
  else
  {
    ROS_INFO_NAMED("tutorial", "Planning failed");
  }

  ros::shutdown();
  return 0;
}