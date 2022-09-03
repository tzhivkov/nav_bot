import rospy
from std_msgs.msg import String, Float64
import signal
import sys
import time


class Navigator(object):
    ''' Simple class that handles all the topics that need to be published to.
        The class makes it easy to expand the functionality of either the
        user inputs or of future autonomous behaviour. Only 2 publish functions
        exist at the moment. One that publishes command inputs to the Idle state
        and the other which sends x&y goal coordinates to the robot object.
    '''

    def __init__(self):
        ''' initialise all the required publishers to control the fsm.

        :@type pub: :class:`rospy.Publisher` manages a (thread-safe) ROS connection
            to a topic. Minimum requirements are a topic name, ROS defined message type,
            queue size

        '''
        self.idle_pub = rospy.Publisher(
            '/idle_input', String, queue_size=3, latch=False)
        self.x_pub = rospy.Publisher(
            '/goal_input_x', Float64, queue_size=3, latch=False)
        self.y_pub = rospy.Publisher(
            '/goal_input_y', Float64, queue_size=3, latch=False)
        rospy.loginfo('\nPublishers initialised')

    def publish_idle_msg(self, idle_msg):
        ''' Publish control messages to the Idle state.
        '''
        pub_msg = '{0} {1}'.format(idle_msg, rospy.get_time())
        rospy.loginfo('idle_msg: {0}'.format(pub_msg))
        self.idle_pub.publish(idle_msg)
        rospy.Rate(10).sleep()

    def publish_xy(self, x_msg, y_msg):
        ''' Publish x&y goal coordinate messages to the GotoGoal state.
            Because two separate messages are published to two topics we use a short
            pause, using rospy.Rate().sleep(), to make sure messages are published successfully.
        '''
        x_pub = '{0} {1}'.format(x_msg, rospy.get_time())
        y_pub = '{0} {1}'.format(y_msg, rospy.get_time())
        rospy.loginfo('x_msg: {0}'.format(x_pub))
        rospy.loginfo('y_msg: {0}'.format(y_pub))
        self.x_pub.publish(x_msg)
        rospy.Rate(10).sleep()
        self.y_pub.publish(y_msg)
        rospy.Rate(10).sleep()


def signal_handler(sig, frame):
    ''' Signal handler to deal with existing the control program preemptively
    '''
    print('\nCaught Ctrl+C!\nExiting')
    sys.exit(0)


def intro_str():
    ''' Clean way of reusing a long string
    '''
    print('\nThis is a simple ROS publisher script.\n\
The only inputs that are accepted are\n\
string-value <q> to quit the program\n\
string-value <h> to send the robot home.\n\
string-value <g> for goal, followed by\n\
float-value <x> for x-coordinate\n\
float-value <y> for y-coordindate\n\
Type <!h> to print this message again!')


def main():
    ''' The main body of code where ROS code is initialised and
        the main loop is used to record input from the command line.
        The nav_msg is the initialised object that publishes user inputs 
        to ROS topics.
    '''
    rospy.init_node('script_navigator', anonymous=True)
    nav_msg = Navigator()
    intro_str()
    signal.signal(signal.SIGINT, signal_handler)
    # Main control loop that takes user input
    while True:
        # Make sure user input is lower case
        user_input = raw_input('command:').lower()
        # The only special case, which requires further user inputs
        if user_input == 'g':
            nav_msg.publish_idle_msg(user_input)
            try:
                x_val = float(raw_input('x:'))
                y_val = float(raw_input('y:'))
            except ValueError:
                print('Wrong data type. Both x and y should be float.')
            nav_msg.publish_xy(x_val, y_val)
        # After quitting the program sleep for 5 seconds to allow the fsm to catch up.
        elif user_input == 'q':
            nav_msg.publish_idle_msg(user_input)
            time.sleep(5)
            sys.exit(0)
        elif user_input == 'h':
            nav_msg.publish_idle_msg(user_input)
        elif user_input == '!h' or user_input == '!H':
            intro_str()
        else:
            print('Unknown command.')
    rospy.spin()


if __name__ == '__main__':
    main()
