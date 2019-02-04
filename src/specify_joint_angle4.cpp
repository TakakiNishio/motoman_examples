#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <iostream>

moveit::planning_interface::MoveGroup::Plan scaling_execution_speed(double speed_scale, moveit::planning_interface::MoveGroup::Plan initial_plan)
{
  moveit_msgs::RobotTrajectory initial_trajectory;
  moveit_msgs::RobotTrajectory new_trajectory;

  initial_trajectory = initial_plan.trajectory_;
  new_trajectory = initial_trajectory;

  int n_joints = initial_trajectory.joint_trajectory.joint_names.size();
  int n_points = initial_trajectory.joint_trajectory.points.size();

  std::cout << "[scaling_execution_speed] speed_scale: " << speed_scale << std::endl;
  std::cout << "[scaling_execution_speed] n_points: " << n_points << std::endl;

  for (int i=1; i<n_points; i++){
    ros::Duration start_time(initial_trajectory.joint_trajectory.points[i].time_from_start.toSec() / speed_scale);
    new_trajectory.joint_trajectory.points[i].time_from_start = start_time;

    for (int j=0; j<n_joints; j++){
      new_trajectory.joint_trajectory.points[i].velocities[j] = initial_trajectory.joint_trajectory.points[i].velocities[j] * speed_scale;
      new_trajectory.joint_trajectory.points[i].accelerations[j] = initial_trajectory.joint_trajectory.points[i].accelerations[j] * speed_scale * speed_scale;
      new_trajectory.joint_trajectory.points[i].positions[j] = initial_trajectory.joint_trajectory.points[i].positions[j];
    }
  }

  moveit::planning_interface::MoveGroup::Plan new_plan;
  new_plan = initial_plan;
  new_plan.trajectory_ = new_trajectory;
  return new_plan;
}

int main(int argc, char** argv)
{
  // Initialization of the ROS node
  ros::init(argc, argv, "moving_the_robot");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialization of moveit
  moveit::planning_interface::MoveGroupInterface group("arm");

  // Setting the start position
  robot_state::RobotState robot_state_start(*group.getCurrentState());
  group.setStartState(robot_state_start);

  // Setting the goal position
  std::map<std::string, double> joints;

  // <group_state name="default" group="arm">
  //   <joint name="joint_1" value="-0.0001745329" />
  //   <joint name="joint_2" value="-0.35203291" />
  //   <joint name="joint_3" value="2.27294228" />
  //   <joint name="joint_4" value="0.0003490659" />
  //   <joint name="joint_5" value="1.22173" />
  //   <joint name="joint_6" value="3.14159" />
  // </group_state>

  // Big move
  // joints["joint_1"] = -0.5952798725345883;
  // joints["joint_2"] = 0.3215349698695835;
  // joints["joint_3"] = 2.065312535177645;
  // joints["joint_4"] = -0.0004594870220877567;
  // joints["joint_5"] = 0.7557963565450452;
  // joints["joint_6"] = 2.54693876500371;

  // Small move
  joints["joint_1"] = 0.5368089499155762;
  joints["joint_2"] = -0.04659610695993788;
  joints["joint_3"] = 2.133536695018872;
  joints["joint_4"] = 0.0009392970585899718;
  joints["joint_5"] = 1.0553840327167352;
  joints["joint_6"] = 3.6782297799277144;

  group.setJointValueTarget(joints);

  // Running the moveit planning
  moveit::planning_interface::MoveGroup::Plan initial_plan;
  group.plan(initial_plan);

  // Scaling the execution speed
  moveit::planning_interface::MoveGroup::Plan new_plan;
  double speed_scale;
  node_handle.param("/speed_scale", speed_scale, 1.5);

  new_plan = scaling_execution_speed(speed_scale, initial_plan);

  // Execute the plan
  group.execute(new_plan);

  ros::spinOnce();
  ros::shutdown();
  return 0;
}
