#! /usr/bin/env python

import rospy
import actionlib
from sys import stdout

import move_base_msgs.msg

def build_nav_goal():
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0
    return goal

def move_base_client():
    print 'Creating client object ...',
    client = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
    print 'done'
    print 'Waiting for server ...',
    stdout.flush()
    client.wait_for_server()
    print 'done'

    goal = build_nav_goal()
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client_py')
        result = move_base_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

