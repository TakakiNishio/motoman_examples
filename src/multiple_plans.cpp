#include <moveit/move_group_interface/move_group.h>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
  // Initialize ROS, create the node handle and an async spinner
  ros::init(argc, argv, "move_group_plan_target");

  ros::AsyncSpinner spin(4);
  spin.start();

  // Get the arm planning group
  moveit::planning_interface::MoveGroup plan_group("arm");
  geometry_msgs::Pose goal;

  int count = 1;

  //first point
  goal.orientation.w = 1.0;
  goal.position.x = 0.6;
  goal.position.y = -0.1;
  goal.position.z = 0.42;

  plan_group.setGoalTolerance(0.2);
  plan_group.setPoseTarget(goal);

  moveit::planning_interface::MoveGroup::Plan goal_plan;
  if (plan_group.plan(goal_plan)){
    // plan_group.move();
    std::size_t size = goal_plan.trajectory_.joint_trajectory.points.size();
    ROS_INFO("Plan%d: length=%d", count, size);
    count += 1;
  }

  sleep(3.0);

  //second point
  goal.orientation.w = 1.0;
  goal.position.x = 0.3;
  goal.position.y = 0.4;
  goal.position.z = 0.22;

  plan_group.setGoalTolerance(0.2);
  plan_group.setPoseTarget(goal);

  if (plan_group.plan(goal_plan)){
    std::size_t size = goal_plan.trajectory_.joint_trajectory.points.size();
    ROS_INFO("Plan%d: length=%d", count, size);
    count += 1;
  }

  sleep(3.0);

  ROS_INFO("Finished all plan!");
  ros::waitForShutdown();
  return 0;
}
