#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
  // Initialization of the ROS node
  ros::init(argc, argv, "moving_the_robot");

  // ros::NodeHandle nh;

  // ros::AsyncSpinner spin(1);
  // spin.start();
  // ros::Publisher display_pub = nh.advertise
  //   <moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);


  // Initialization of moveit
  moveit::planning_interface::MoveGroup group("arm");


  // Setting the start position
  robot_state::RobotState robot_state_start(*group.getCurrentState());
  // geometry_msgs::Pose start_pose;

  // start_pose.orientation.w = 1.0;
  // start_pose.position.x = 0.45;
  // start_pose.position.y = 0.1;
  // start_pose.position.z = 0.65;

  // const robot_state::JointModelGroup* joint_model_group =
  //   robot_state_start.getJointModelGroup(group.getName());

  // robot_state_start.setFromIK(joint_model_group, start_pose);
  // group.setStartState(robot_state_start);

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

  group.setGoalTolerance(0.2);

  group.setPoseTarget(target);

  // std::vector<int> v = {3, 1, 4, 5, 2};
  // std::size_t size = v.size();
  // std::cout << size << std::endl;

  // Running the moveit planning
  moveit::planning_interface::MoveGroup::Plan goal_plan;
  if (group.plan(goal_plan))
  {
    group.move();
  }

  // moveit::planning_interface::MoveGroup::Plan result_plan;

  // bool success = group.plan(result_plan);

  // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);

  if (group.plan(goal_plan)){
    group.move();
    std::size_t size = 1;
    // std::size_t size = result_plan.trajectory_.joint_trajectory.points.size() + 1;
    std::cout << size << std::endl;
  }

  // // group.move();
  ros::shutdown();
  return 0;
}
