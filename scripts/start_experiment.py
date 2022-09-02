from operator import truediv
from xmlrpc.client import Boolean
import rospy
from std_msgs.msg import String, Float64, Bool
import signal
import sys
import time


class Navigator(object):
    def __init__(self):
        ''' initialise all the required publishers to control the fsm
        '''
        self.idle_pub = rospy.Publisher('/idle_input', String, queue_size=3, latch=False)
        self.x_pub = rospy.Publisher('/goal_input_x', Float64, queue_size=3, latch=False)
        self.y_pub = rospy.Publisher('/goal_input_y', Float64, queue_size=3, latch=False)
        rospy.loginfo('\nPublishers initialised')


    def publish_idle_msg(self, idle_msg):
        pub_msg = '{0} {1}'.format(idle_msg, rospy.get_time())
        rospy.loginfo('idle_msg: {0}'.format(pub_msg))
        self.idle_pub.publish(idle_msg)
        rospy.Rate(10).sleep()


    def publish_xy(self, x_msg, y_msg):
        x_pub = '{0} {1}'.format(x_msg, rospy.get_time())
        y_pub = '{0} {1}'.format(y_msg, rospy.get_time())
        rospy.loginfo('x_msg: {0}'.format(x_pub))
        rospy.loginfo('y_msg: {0}'.format(y_pub))
        self.x_pub.publish(x_msg)
        rospy.Rate(10).sleep()
        self.y_pub.publish(y_msg)
        rospy.Rate(10).sleep()


def signal_handler(sig, frame):
    print('\nCaught Ctrl+C!\nExiting')
    sys.exit(0)


def intro_str():
    print('\nThis is a simple ROS publisher script.\n\
The only inputs that are accepted are\n\
string-value <q> to quit the program\n\
string-value <h> to send the robot home.\n\
string-value <g> for goal, followed by\n\
float-value <x> for x-coordinate\n\
float-value <y> for y-coordindate\n\
Type <!h> to print this message again!')


def main():
    '''
    '''
    rospy.init_node('script_navigator', anonymous=True)
    nav_msg = Navigator()
    intro_str()
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        user_input = raw_input('command:').lower()
        if user_input == 'g':
            nav_msg.publish_idle_msg(user_input)
            try:
                x_val = float(raw_input('x:'))
                y_val = float(raw_input('y:'))
            except ValueError:
                print('Wrong data type. Both x and y should be float.')
                sys.exit(0)
            nav_msg.publish_xy(x_val,y_val)
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