#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <iostream>

int main(int argc, char** argv)
{
  // Initialization of the ROS node
  ros::init(argc, argv, "moving_the_robot");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialization of moveit
  moveit::planning_interface::MoveGroupInterface group("arm");

  // group.setPlannerId("STOMP");

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

  // joints["joint_1"] = -0.785398163;
  // joints["joint_2"] = -0.35203291;
  // joints["joint_3"] = 2.27294228;
  // joints["joint_4"] = 0.0003490659;
  // joints["joint_5"] = 1.22173;
  // joints["joint_6"] = 3.14159;

  joints["joint_1"] = -0.5952798725345883;
  joints["joint_2"] = 0.3215349698695835;
  joints["joint_3"] = 2.065312535177645;
  joints["joint_4"] = -0.0004594870220877567;
  joints["joint_5"] = 0.7557963565450452;
  joints["joint_6"] = 2.54693876500371;

  group.setJointValueTarget(joints);

  // Running the moveit planning
  moveit::planning_interface::MoveGroup::Plan initial_plan;
  group.plan(initial_plan);

  moveit_msgs::RobotTrajectory initial_trajectory;
  moveit_msgs::RobotTrajectory new_trajectory;

  initial_trajectory = initial_plan.trajectory_;
  new_trajectory = initial_trajectory;
  new_trajectory.joint_trajectory.header = initial_trajectory.joint_trajectory.header;
  // new_trajectory.multi_dof_joint_trajectory = initial_trajectory.multi_dof_joint_trajectory;

  int n_joints = 6;
  // int n_points = sizeof(initial_trajectory.joint_trajectory.points);
  int n_points = initial_trajectory.joint_trajectory.points.size();
  double speed_scale = 3.0;

  std::cout << n_points << std::endl;

  // ros::Duration start_time_0(initial_trajectory.joint_trajectory.points[0].time_from_start.toSec());

  for (int i=0; i<n_points; i++){
    ros::Duration start_time(initial_trajectory.joint_trajectory.points[i].time_from_start.toSec() / speed_scale);
    new_trajectory.joint_trajectory.points[i].time_from_start = start_time;

    for (int j=0; j<n_joints; j++){
      new_trajectory.joint_trajectory.points[i].velocities[j] = initial_trajectory.joint_trajectory.points[i].velocities[j] * speed_scale;
      // new_trajectory.joint_trajectory.points[i].accelerations[j] = initial_trajectory.joint_trajectory.points[i].accelerations[j] * speed_scale * speed_scale;
      // new_trajectory.joint_trajectory.points[i].velocities[j] = initial_trajectory.joint_trajectory.points[i].velocities[j];
      new_trajectory.joint_trajectory.points[i].accelerations[j] = initial_trajectory.joint_trajectory.points[i].accelerations[j];
      new_trajectory.joint_trajectory.points[i].positions[j] = initial_trajectory.joint_trajectory.points[i].positions[j];
    }
  }

  // initial_trajectory.joint_trajectory.points[0].time_from_start = start_time_0;

  moveit::planning_interface::MoveGroup::Plan new_plan;
  new_plan.planning_time_ = initial_plan.planning_time_;
  new_plan.start_state_ = initial_plan.start_state_;
  new_plan.trajectory_ = new_trajectory;

  // group.execute(initial_plan);
  group.execute(new_plan);
  // group.execute(new_trajectory);

  ros::spinOnce();
  ros::shutdown();
  return 0;
}
