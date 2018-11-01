#include <moveit/move_group_interface/move_group.h>

int main(int argc, char** argv)
{
  // Initialization of the ROS node
  ros::init(argc, argv, "moving_the_robot");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialization of moveit
  moveit::planning_interface::MoveGroup group("arm");

  // // Setting the start position
  group.setStartState(*group.getCurrentState());

  // Setting the goal position
  geometry_msgs::Pose goal;
  goal.position.x = 0.5557;
  goal.position.y = 0.153236;
  goal.position.z = 0.353918;
  goal.orientation.w = 8.55919e-5;
  goal.orientation.x = 0.713737;
  goal.orientation.y = -0.000233858;
  goal.orientation.z = 0.700414;

  group.setPoseTarget(goal);

  // To use the STOMP planner
  robot_state::RobotState robot_state_goal(*group.getCurrentState());
  std::map<std::string, double> joints;
  joints["joint_1"] = robot_state_goal.getVariablePosition("joint_1");
  joints["joint_2"] = robot_state_goal.getVariablePosition("joint_2");
  joints["joint_3"] = robot_state_goal.getVariablePosition("joint_3");
  joints["joint_4"] = robot_state_goal.getVariablePosition("joint_4");
  joints["joint_5"] = robot_state_goal.getVariablePosition("joint_5");
  joints["joint_6"] = robot_state_goal.getVariablePosition("joint_6");
  group.setJointValueTarget(joints);

  // Running the moveit planning
  moveit::planning_interface::MoveGroup::Plan result_plan;
  group.plan(result_plan);

  return 0;
}
