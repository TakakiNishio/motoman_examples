#include <moveit/move_group_interface/move_group.h>

int main(int argc, char** argv)
{
  // Initialization of the ROS node
  ros::init(argc, argv, "moving_the_robot");

  // Initialization of moveit
  moveit::planning_interface::MoveGroup group("arm");

  // Setting the start position
  // group.setStartState(*group.getCurrentState());

  // Setting the goal position
  geometry_msgs::Pose target;
  target.orientation.w = 1.0;
  target.position.x = 0.6;
  target.position.y = -0.1;
  target.position.z = 0.42;

  group.setPoseTarget(target);

  // To use the STOMP planner
  robot_state::RobotState robot_state_goal(*group.getCurrentState());
  std::map<std::string, double> joints;
  joints["joint_s"] = robot_state_goal.getVariablePosition("joint_s");
  joints["joint_l"] = robot_state_goal.getVariablePosition("joint_l");
  joints["joint_e"] = robot_state_goal.getVariablePosition("joint_e");
  joints["joint_u"] = robot_state_goal.getVariablePosition("joint_u");
  joints["joint_r"] = robot_state_goal.getVariablePosition("joint_r");
  joints["joint_b"] = robot_state_goal.getVariablePosition("joint_b");
  joints["joint_t"] = robot_state_goal.getVariablePosition("joint_t");
  group.setJointValueTarget(joints);

  // Running the moveit planning
  moveit::planning_interface::MoveGroup::Plan result_plan;
  group.plan(result_plan);

  return 0;
}
