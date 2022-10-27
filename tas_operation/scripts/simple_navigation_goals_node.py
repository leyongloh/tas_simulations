#!/usr/bin/env python3
"""
This node sends a sequence of goals to the move_base, which guide the TAS car through the basement of the
N5 building. Therefore, it uses the actionlib package, which provides tools to create servers that execute
long-running goals that can be preempted. For more information about the actionlib package refer to this:

http://wiki.ros.org/actionlib

Reference for terminal status values:
http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveBaseSeq():

    def __init__(self):
        # Constructor
        rospy.init_node('simple_navigation_goals')
        rospy.loginfo('simple_navigation_goals_node started')

        # variable definitions
        self.goal_counter = 0  # goal counter
        self.active_goal = MoveBaseGoal()  # MoveBaseGoal msg
        self.pose_seq = [[-2.00, 2.00, 0.53, 0.85],  # pose 1  # 2D-list of goal poses
                         [-1.00, 11.50, -0.98, 0.22],  # pose 2
                         [-6.27, 10.17, -0.86, 0.50],  # pose 3
                         [-0.22, 5.82,  -0.84, 0.54],  # pose 4
                         [-0.72, -6.3,  -0.84, 0.54]]  # pose 5

        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))  # waiting 5 seconds until action server is up
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return  # cancel if action server is not available
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        # to print current pose when getting active
        rospy.loginfo("Goal pose " + str(self.goal_counter + 1) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # to print current pose at each feedback:
        rospy.loginfo("Feedback for goal " + str(self.goal_counter) + ": " + str(feedback))
        rospy.loginfo("Feedback for goal pose " + str(self.goal_counter + 1) + " received")

    def done_cb(self, status, result):
        # callback when action server finished current goal due to following reasons in the if statements
        self.goal_counter += 1  # prepare next goal

        if status == 2:  # cancel goal
            rospy.loginfo("Goal pose " + str(
                self.goal_counter) + " received a cancel request after it started executing, completed execution!")

        if status == 3:  # goal successfully achieved
            rospy.loginfo("Goal pose " + str(self.goal_counter) + " reached")
            if self.goal_counter < len(self.pose_seq):  # still goals left
                self.update_goal()  # update active_goal with next goal pose
                rospy.loginfo("Sending goal pose " + str(self.goal_counter + 1) + " to Action Server")
                rospy.loginfo(str(self.active_goal))
                self.client.send_goal(self.active_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:  # all goals reached
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:  # goal aborted
            rospy.loginfo("Goal pose " + str(self.goal_counter) + " was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_counter) + " aborted, shutting down!")
            return

        if status == 5:  # goal rejected
            rospy.loginfo("Goal pose " + str(self.goal_counter) + " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_counter) + " rejected, shutting down!")
            return

        if status == 8:  # goal cancelled
            rospy.loginfo("Goal pose " + str(
                self.goal_counter) + " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        # core method with ros spin()
        self.update_goal()  # initializing active_goal with first pose
        rospy.loginfo("Sending goal pose " + str(self.goal_counter + 1) + " to Action Server")
        rospy.loginfo(str(self.active_goal))
        self.client.send_goal(self.active_goal, self.done_cb, self.active_cb, self.feedback_cb)  # send first goal
        rospy.spin()

    def update_goal(self):
        # updating active_goal with next pose
        self.active_goal.target_pose.header.frame_id = "map"
        self.active_goal.target_pose.header.stamp = rospy.Time.now()
        self.active_goal.target_pose.pose.position.x = self.pose_seq[self.goal_counter][0]
        self.active_goal.target_pose.pose.position.y = self.pose_seq[self.goal_counter][1]
        self.active_goal.target_pose.pose.position.z = 0.0
        self.active_goal.target_pose.pose.orientation.x = 0.0
        self.active_goal.target_pose.pose.orientation.y = 0.0
        self.active_goal.target_pose.pose.orientation.z = self.pose_seq[self.goal_counter][2]
        self.active_goal.target_pose.pose.orientation.w = self.pose_seq[self.goal_counter][3]


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
