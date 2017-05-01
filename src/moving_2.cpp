#include <moveit/move_group_interface/move_group.h>

int main(int argc, char** argv)
{
  // Initialization of the ROS node
  ros::init(argc, argv, "moving_the_robot");


  // Initialization of moveit
  moveit::planning_interface::MoveGroup group("arm");


  // Setting the start position
  robot_state::RobotState robot_state_start(*group.getCurrentState());
  geometry_msgs::Pose start_pose;

  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.45;
  start_pose.position.y = 0.1;
  start_pose.position.z = 0.65;

  const robot_state::JointModelGroup *joint_model_group =
    robot_state_start.getJointModelGroup(group.getName());

  robot_state_start.setFromIK(joint_model_group, start_pose);
  group.setStartState(robot_state_start);

  // Setting the start position(another way to do)
  // robot_state::RobotState robot_state_start(*group.getCurrentState());

  // std::vector<double> start_joints(7);

  // for(int i=0;i<7;i++){
  //   start_joints[i] = 0.2;
  // }

  // robot_state_start.setVariablePositions(start_joints);
  // group.setStartState(robot_state_start);


  // Setting the goal position
  geometry_msgs::Pose target;
  target.orientation.w = 1.0;
  target.position.x = 0.6;
  target.position.y = -0.1;
  target.position.z = 0.42;

  group.setPoseTarget(target);


  // Running the moveit planning
  moveit::planning_interface::MoveGroup::Plan result_plan;
  group.plan(result_plan);

  return 0;
}
