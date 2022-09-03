#!/usr/bin/env python

import smach
import smach_ros
import rospy
from nav_bot import Robot
from std_msgs.msg import String, Float64

RATE = 10  # Hz


# Smach State Machines - Start
class Start(smach.State):
    ''' Class representing the Start state of the FSM.
        Checks to make sure the robot object is ready to initialise navigation.

        :param tb3_robot: passes the mutable :class:`nav_bot.Robot` object
        :type tb3_robot: :class:`nav_bot.Robot`
    '''

    def __init__(self, tb3_robot):
        smach.State.__init__(self, outcomes=['start'])
        self.tb3_robot = tb3_robot

    def execute(self, userdata):
        ''' Called when executing a state. The outcome 'start' moves the Start state to the Idle state.

            :param userdata: Part of the :class:`smach.State` used to pass input/output keys between states
            :type userdata: UserData
            :return: The next state transition
            :rtype: str

        '''
        rospy.loginfo('state: Start')
        if self.tb3_robot.c_state == 'ready':
            return 'start'


class Stop(smach.State):
    ''' Class representing the Stop state of the FSM.
        Called when user inputs <q> or when rostopic /idle_input receives 'q' string.

        :param tb3_robot: Passes the mutable :class:`nav_bot.Robot` object
        :type tb3_robot: :class:`nav_bot.Robot`
    '''

    def __init__(self, tb3_robot):
        smach.State.__init__(self, outcomes=['shutdown'])
        self.tb3_robot = tb3_robot

    def execute(self, userdata):
        ''' Called when executing a state. The outcome 'shutdown' stops the introspective server.

            :param userdata: Part of the :class:`smach.State` used to pass input/output keys between states
            :type userdata: UserData
            :return: The next state transition
            :rtype: str
        '''
        self.tb3_robot.shutdown()
        rospy.loginfo('### state: Stop ###')
        return 'shutdown'


class Moving(smach.State):
    ''' Class representing the Moving state of the FSM.
        Monitors the robot object (tb3_robot) to know when it has completed a goal.

        :param tb3_robot: Passes the mutable :class:`nav_bot.Robot` object
        :type tb3_robot: :class:`nav_bot.Robot`
    '''

    def __init__(self, tb3_robot):
        smach.State.__init__(self, outcomes=['arrived'])
        self.tb3_robot = tb3_robot

    def execute(self, userdata):
        ''' Called when executing a state. The outcome 'arrived' always goes back to 
            the Idle state.

            :param userdata: Part of the :class:`smach.State` used to pass input/output 
                keys between states
            :type userdata: UserData
            :return: The next state transition
            :rtype: str
        '''
        rospy.loginfo('state: Moving')
        self.tb3_robot.goal_outcome()
        # Check to make sure tb3_robot object has received the goal
        while self.tb3_robot.c_state == 'goal_sent':
            rospy.Rate(RATE).sleep()
        # Monitor the success of the goal and alart the user
        if self.tb3_robot.c_state == 'success':
            return 'arrived'
        elif self.tb3_robot.c_state == 'aborted':
            rospy.loginfo('Goal was rejected or aborted beacuse it was outside the bounds\
                          of the map or in an obstacle.')
            return 'arrived'
        elif self.tb3_robot.c_state == 'fail':
            rospy.loginfo(
                'Goal was either too far or obstacle prevented robot from reaching it.')
            return 'arrived'


class GotoGoal(smach.State):
    ''' Class representing the GotoGoal state of the FSM.
        Subcribes to the x and y coordinate rostopics with callback functions and monitors 
        that user input is reasonable.

        :param tb3_robot: Passes the mutable :class:`nav_bot.Robot` object
        :type tb3_robot: :class:`nav_bot.Robot`
    '''

    def __init__(self, tb3_robot):
        smach.State.__init__(self, outcomes=['sent', 'gotogoal'])
        self._clear_vars()
        self.tb3_robot = tb3_robot
        rospy.Subscriber('/goal_input_x', Float64, self.callback_x)
        rospy.Subscriber('/goal_input_y', Float64, self.callback_y)

    def callback_x(self, data):
        ''' Rostopic callback recording the x-value of the user defined goal

            :param callback: Handles ROS callback method
            :type callback: function, optional
        '''
        self.x = float(data.data)
        rospy.loginfo('{0} user input: {1}'.format(
            rospy.get_caller_id(), self.x))

    def callback_y(self, data):
        ''' Rostopic callback recording the y-value of the user defined goal

            :param callback: Handles ROS callback method
            :type callback: function, optional
        '''
        self.y = float(data.data)
        rospy.loginfo('{0} user input: {1}'.format(
            rospy.get_caller_id(), self.y))

    def _clear_vars(self):
        ''' Variables are cleared using a function, as smach.State.__init__() is called 
            once at the initialisation of the script.
        '''
        # some very high value
        self.x = 9999.0
        self.y = 9999.0

    def execute(self, userdata):
        ''' Called when executing a state. Call back state GotoGoal if user has broken 
            the rules for x/y value range (which is already higher than it should be).
            Passes the received x and y coordinates to the tb3_robot object and clears variables.

            :param userdata: Part of the :class:`smach.State` used to pass input/output 
                keys between states
            :type userdata: UserData
            :return: The next state transition
            :rtype: str
        '''
        rospy.loginfo('state: GotoGoal')
        # Wait until both values are reasonable
        while self.x == 9999.0 or self.y == 9999.0:
            rospy.Rate(RATE).sleep()

        # Limit user input ranges
        if self.x > 20.0 or self.y > 20.0 or \
                self.x < -20.0 or self.y < -20.0:
            rospy.loginfo(
                'Controller has received x or y values that are smaller/greater than -20/20.')
            return 'gotogoal'

        pos = {'x': self.x, 'y': self.y}
        quater = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}
        self.tb3_robot.goto_pose(pos, quater)
        if self.tb3_robot.c_state == 'goal_sent':
            self._clear_vars()
            return 'sent'


class Idle(smach.State):
    ''' Class representing the Idle state of the FSM.
        Subcribes to the Idle input rostopic with callback function which is used
        to give the tb3_robot instructions. The self.instruct variable, used to
        record user inputs, needs to be cleared at each change of states.
        The user input decides the next state after Idle.

        :param tb3_robot: Passes the mutable :class:`nav_bot.Robot` object
        :type tb3_robot: :class:`nav_bot.Robot`
    '''

    def __init__(self, tb3_robot):
        smach.State.__init__(
            self, outcomes=['received_task', 'shutdown', 'goto_home', 'idle'])
        self._clear_vars()
        self.tb3_robot = tb3_robot
        rospy.Subscriber('/idle_input', String, self.callback)

    def callback(self, data):
        ''' Rostopic callback recording user input and storing in self.instruct

            :param callback: Handles ROS callback method
            :type callback: function, optional
        '''
        self.intstruct = str(data.data)
        rospy.loginfo('{0} user input: {1}'.format(
            rospy.get_caller_id(), self.intstruct))

    def _clear_vars(self):
        ''' Variables are cleared using a function, as smach.State.__init__() is called 
            once at the initialisation of the script.
        '''
        self.intstruct = ''

    def execute(self, userdata):
        ''' Called when executing a state. Three states can be reached from Idle.
            Idle can reach GotoGoal, Moving and Shutdown. When transitioing to
            'goto_home' Idle goes directly to Moving.

            :return: The next state transition
            :rtype: str
        '''
        rospy.loginfo('state: Idle')
        self.tb3_robot.clear()
        rospy.loginfo('waiting for user input on topic \'/idle_input\' <q: quit,\
                      g: goal location, h: goto home location>')
        # If no instruction exists, the tb3_robot object sleeps
        while self.intstruct == '':
            rospy.Rate(RATE).sleep()

        # After a user command is accepted, clear variables and switch states
        if self.intstruct == 'q':
            self._clear_vars()
            return 'shutdown'
        elif self.intstruct == 'g':
            self._clear_vars()
            return 'received_task'
        elif self.intstruct == 'h':
            self.tb3_robot.go_home()
            self._clear_vars()
            return 'goto_home'
        else:
            rospy.loginfo('Incorrect input was given. Try again.')
            self._clear_vars()
            return 'idle'


def main():
    ''' Initialises the main body of code. All ROS setup and initialisation also takes 
        place here. The states of the fsm are added here and each state transition is defined.    
    '''
    node_name = 'fsm_controller'
    rospy.loginfo("Starting node '{0}'...".format(node_name))
    rospy.init_node(node_name)
    tb3_robot = Robot()
    # Unique server name
    introspecServer = 'tb3_server'
    # initiate smach state machine (sm)
    sm_top = smach.StateMachine(outcomes=['done'])

    with sm_top:
        smach.StateMachine.add('START', Start(tb3_robot),
                               transitions={'start': 'IDLE'})
        smach.StateMachine.add('IDLE', Idle(tb3_robot),
                               transitions={'received_task': 'GOTOGOAL',
                                            'shutdown': 'SHUTDOWN',
                                            'goto_home': 'MOVING',
                                            'idle': 'IDLE'})
        smach.StateMachine.add('GOTOGOAL', GotoGoal(tb3_robot),
                               transitions={'sent': 'MOVING',
                                            'gotogoal': 'GOTOGOAL'})
        smach.StateMachine.add('MOVING', Moving(tb3_robot),
                               transitions={'arrived': 'IDLE'})
        smach.StateMachine.add('SHUTDOWN', Stop(tb3_robot),
                               transitions={'shutdown': 'done'})
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(introspecServer, sm_top, '/SM_ROOT_1')
    sis.start()

    outcome = sm_top.execute()
    sis.stop()
    rospy.spin()


if __name__ == '__main__':
    main()
