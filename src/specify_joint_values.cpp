#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "move_group_plan_single_target");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // Get the arm planning group
    moveit::planning_interface::MoveGroup plan_group("arm");

    // Create a published for the arm plan visualization
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    plan_group.setStartState(*plan_group.getCurrentState());

    // Setting the goal position
    std::map<std::string, double> joints;

    joints["joint_s"] = -0.8;
    joints["joint_l"] = 0.2;
    joints["joint_e"] = 0.0;
    joints["joint_u"] = -0.4;
    joints["joint_r"] = 0.35;
    joints["joint_b"] = 0.6;
    joints["joint_t"] = 0.4;

    plan_group.setJointValueTarget(joints);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    }


    ROS_INFO("Finished all plan and move!");
    ros::waitForShutdown();
    // ros::shutdown();

    return 0;
}
