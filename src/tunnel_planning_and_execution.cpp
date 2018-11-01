#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>


int main(int argc, char **argv)
{
  // Initialize ROS, create the node handle and an async spinner
  ros::init(argc, argv, "moving_the_robot");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get the arm planning group
  moveit::planning_interface::MoveGroup group("arm");

  // Setting the start position
  robot_state::RobotState robot_state_start(*group.getCurrentState());
  group.setStartState(robot_state_start);

  // Create a published for the arm plan visualization
  ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  //Roll pitch and yaw in Radians
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);  // Create this quaternion from roll/pitch/yaw (in radians)
  geometry_msgs::Quaternion quaternion_msg;
  quaternionTFToMsg(quaternion, quaternion_msg);
  ROS_INFO("x: %f", quaternion_msg.x);
 
  // Set a goal message as a pose of the end effector
  geometry_msgs::Pose goal;
  goal.position.x = 0.5557;
  goal.position.y = 0.153236;
  goal.position.z = 0.353918;
  goal.orientation.w = 8.55919e-5;
  goal.orientation.x = 0.713737;
  goal.orientation.y = -0.000233858;
  goal.orientation.z = 0.700414;

  // goal.orientation.w = -0.000113224;
  // goal.orientation.x = 0.700419;
  // goal.orientation.y = -0.000485293;
  // goal.orientation.z = 0.700419;

  // Set the tolerance to consider the goal achieved
  group.setGoalTolerance(0.001);

  // Set the target pose, which is the goal we already defined
  group.setPoseTarget(goal);

  // Perform the planning step, and if it succeeds display the current
  // arm trajectory and move the arm
  moveit::planning_interface::MoveGroup::Plan goal_plan;
  if (group.plan(goal_plan))
  {
      moveit_msgs::DisplayTrajectory display_msg;
      display_msg.trajectory_start = goal_plan.start_state_;
      display_msg.trajectory.push_back(goal_plan.trajectory_);
      display_pub.publish(display_msg);

      sleep(5.0);

      group.move();
  }

  sleep(2.0);

  // Setting the start position
  group.setStartState(*group.getCurrentState());

  goal.position.x = 0.5557;
  goal.position.y = 0;
  goal.position.z = 0.5953;
  goal.orientation.w = 8.55919e-5;
  goal.orientation.x = 0.713737;
  goal.orientation.y = -0.000233858;
  goal.orientation.z = 0.700414;

  // Set the tolerance to consider the goal achieved
  group.setGoalTolerance(0.001);

  // Set the target pose, which is the goal we already defined
  group.setPoseTarget(goal);

  // Perform the planning step, and if it succeeds display the current
  // arm trajectory and move the arm
  moveit::planning_interface::MoveGroup::Plan goal_plan2;
  if (group.plan(goal_plan2))
  {
      moveit_msgs::DisplayTrajectory display_msg;
      display_msg.trajectory_start = goal_plan2.start_state_;
      display_msg.trajectory.push_back(goal_plan2.trajectory_);
      display_pub.publish(display_msg);

      sleep(5.0);

      group.move();
  }

  ROS_INFO("Finished all plan and move!");
  ros::waitForShutdown();
  // ros::shutdown();

  return 0;
}
