#!/usr/bin/env python

# ROS specific libraries
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy


class Robot(object):
    ''' Robot class that creates messages to send to move_base and monitors the goal status.
        The class allows for the control over goal-seeking behaviour and navigation. It is
        intended to be used with a state machine or some other control code.
    '''

    def __init__(self):
        self.goal_sent = False
        # The c_state variable internally monitors the robot object it can be used by the
        # intended control code
        self.c_state = ''
        # Tell the action client that we want to spin a thread
        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        # Call initial private methods
        self._update_state()
        self._check_start()

    def _check_start(self):
        ''' Function makes sure that the robot object is ready. Updates the
            c_state of the robot object.
        '''
        self.c_state = 'ready'

    def _update_state(self):
        ''' A future method that will be used more often if the robot object
            becomes more complex.

            :return: :class:`SimpleActionClient.get_state()`
            :rtype: int
        '''
        return self.move_base.get_state()

    def clear(self):
        ''' Future function that will be used for clearing the costmap manually,
            rather than only the planner clearing them.
            For now this function only updates the c_state to tell the control code
            that the costmaps are clear and it is ready for a new goal location.
        '''
        self.c_state = 'clear'

    def go_home(self):
        ''' Returns the robot to any coordinates that are given in this function.
            Currently the robot will go back to starting location, if it is already there
            it will do nothing.
            TODO: In the future these values should be defined at the initialisation of the 
            robot object and passed to this function (like in goto_pose) instead of hard coding them.
        '''
        rospy.loginfo('Robot is moving to new pose')
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(-2.0, -0.5, 0.000),
                                     Quaternion(0.000, 0.000, 0.000, 1.000))
        self.move_base.send_goal(goal)
        self.c_state = 'goal_sent'

    def goto_pose(self, pos, quater):
        ''' Responsible for sending a goal to move_base. Updates the internal
            c_state of the robot object.

            :param pos: Dictionary defining the location of a goal
            :type pos: dict of str: float
            :param quater: Dictionary of quaternion allowing to control 3 dimensions of the robot
                e.g., heading.
            :type quater: dict of str: float
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
        ''' Monitors the robot's progress and updates the c_state accordingly.
            It is important to cancel the goal only in certain GoalStatus
            to prevent errors with move_base, as shown in this funtion.

        '''
        # If the robot hasn't reached the goal in 60 seconds then goal will be cancelled
        time_out = self.move_base.wait_for_result(rospy.Duration(60))
        state = self._update_state()

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
        ''' Cancel goal when shutdown is called.
        '''
        # if the robot object receives shutdown while it has a goal, it will cancel it first
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
