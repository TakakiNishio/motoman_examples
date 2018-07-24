#!/usr/bin/env python
import rospy
from moveit_msgs.msg import MoveGroupActionResult

class ResultProcessingNode:

    def __init__(self):
        rospy.init_node('result_listener', anonymous=False)
        rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.callback)

    def callback(self,data):
        # rospy.loginfo('I heard')
        # rospy.loginfo(data)
        # rospy.loginfo('I heard!!!!!!!!!!!!!!!!!!')
        rospy.loginfo(data.result.planned_trajectory.joint_trajectory.joint_names)
        # rospy.loginfo(data.result.planned_trajectory.joint_trajectory.points)

        points = data.result.planned_trajectory.joint_trajectory.points

        for point in points:
            print(point.positions)
            print(len(point.positions))

if __name__ == '__main__':
    ResultProcessingNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
