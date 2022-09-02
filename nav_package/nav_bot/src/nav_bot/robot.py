#!/usr/bin/env python

'''robot.py

A robot class that subscribes and publishes to all the important navigation stack topics. The class allows for control over goal-seeking behaviour and navigation. The class is intended to be used with a state machine or some other control code.

Usage: robot.py [robot_name]
    robot_name: the ROS node name given to the robot.
'''

# Standard libraries
import datetime
import math
import sys
import time

# ROS specific libraries
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

# nav_bot libraries
from nav_bot.utility import stamp


class Robot(object):
    '''
    A robot class
    '''
    def __init__(self):
        '''
            docstring
        '''
        self.goal_sent = False
        self.c_state = ''
        # Tell the action client that we want to spin a thread by default
        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        self.update_state()
        self.check_start()


    def check_start(self):
        self.c_state = 'ready'


    def clear(self):
        self.c_state = 'clear'

    def update_state(self):
        return self.move_base.get_state()

    def go_home(self):
        ''' Will go back to starting location
        '''
        rospy.loginfo('Robot is moving to new pose')
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(-2.0,-0.5, 0.000),
                                     Quaternion(0.000, 0.000, 0.000, 1.000))
        self.move_base.send_goal(goal)
        self.c_state = 'goal_sent'


    def goto_pose(self, pos, quater):
        '''
            Send goal
        '''
        rospy.loginfo('Robot is moving to new pose')
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quater['r1'], quater['r2'], quater['r3'], quater['r4']))
        self.move_base.send_goal(goal)
        self.c_state = 'goal_sent'

    def goal_outcome(self):
        '''
            Goal outcome
        '''
        time_out = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.update_state()
        if (time_out and state == GoalStatus.SUCCEEDED):
            self.c_state = 'success'
        elif state == GoalStatus.REJECTED or state == GoalStatus.ABORTED:
            self.move_base.cancel_goal()
            self.c_state = 'aborted'
        else:            
            self.move_base.cancel_goal()
            self.c_state = 'fail'
        self.goal_sent = False


    def shutdown(self):
        ''' 
            Cancel goal if Ctrl+C is captured
        '''
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)