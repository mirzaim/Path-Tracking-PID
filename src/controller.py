#! /usr/bin/env python3

# Import the Python library for ROS
import rospy
import time
import random

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from itertools import product, count

# Import the Twist message
from geometry_msgs.msg import Twist, Point
from math import radians, atan2, sqrt, sin, cos, pi, floor


class TurtleController():

    def __init__(self):
        # Initiate a named node
        rospy.init_node('TurtleController', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("CTRL + C to stop the turtlebot")

        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        self.t = True
        # Creates a var of msg type Twist for velocity
        self.vel = Twist()
        self.cur_pos = Odometry()

        self.goal_pos = Point()
        self.goal_tensor = {'x': 0, 'y': 0, 'slope': 0,
                            'd': 0, 'dt': 1e-16, 'prev_t': 0}

        # Create a Publisher object, will publish on /cmd_vel topic
        # to which the robot (real or simulated) is a subscriber
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goal_pub = rospy.Publisher('/turtle_goal', Point, queue_size=1)

        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.callback_odometry)

        # Set a publish velocity rate of in Hz
        self.rate = rospy.Rate(10)

    def callback_odometry(self, msg):
        if self.t:
            self.cur_pos = msg
            self.goal_tensor['x'] = self.goal_pos.x - msg.pose.pose.position.x
            self.goal_tensor['y'] = self.goal_pos.y - msg.pose.pose.position.y
            self.goal_tensor['d'] = sqrt(
                self.goal_tensor['x']**2 + self.goal_tensor['y']**2)
            self.goal_tensor['dt'] = rospy.get_time() - \
                self.goal_tensor['prev_t']
            self.goal_tensor['prev_t'] = rospy.get_time()
            self.goal_tensor['slope'] = atan2(
                self.goal_tensor['y'], self.goal_tensor['x'])

    def ellipse_param(self, a, b, t):
        return a*cos(t), b*sin(t)

    def pd_circle_mv(self, r, velocity, dstar, c1, c2, c3, c4):
        self.stop()
        parts = 16  # floor(2*pi*r/dstar)
        w = (2*pi)/parts
        prev_deff = 0
        integral = []
        mean = self.cur_pos.twist.twist.angular.z
        for t in range(parts):
            self.goal_pos.x = r*cos(w*t-pi/2)
            self.goal_pos.y = r*sin(w*t-pi/2) + r
            self.goal_pub.publish(self.goal_pos)
            print(self.goal_pos)
            self.t = True
            self.rate.sleep()

            while not (self.goal_tensor['d'] < dstar):
                thetha = self.cur_pos.pose.pose.orientation.z
                self.vel.linear.x = velocity
                integral.append(self.goal_tensor['slope'])
                mean = 0.25 * self.cur_pos.twist.twist.angular.z + 0.75 * mean
                self.vel.angular.z = c1*(self.goal_tensor['slope'] - thetha) + \
                    -1*c2*self.goal_tensor['slope'] + \
                    c3*(self.goal_tensor['slope'] - thetha - prev_deff)/self.rate.sleep_dur.to_sec() + \
                    c4*(sum(integral)/len(integral))*self.rate.sleep_dur.to_sec() + \
                    mean
                self.vel_pub.publish(self.vel)
                prev_slope = self.goal_tensor['slope'] - thetha
                self.t = True
                self.rate.sleep()

    def pi_ellipse_mv(self, a, b, c, alpha, dstar, Kp, Ki, Tp, int_max):
        parts = 32
        w = (2*pi)/parts
        proportional = 0
        integral = 0
        goals = 0
        for t in count():
            self.goal_pos.x = a*cos(w*t - alpha)
            self.goal_pos.y = b*sin(w*t - alpha) + c
            print(f'goal is ({self.goal_pos.x}, {self.goal_pos.y})')
            self.goal_pub.publish(self.goal_pos)
            self.t = True
            self.rate.sleep()

            while not (self.goal_tensor['d'] < dstar):
                if self.goal_tensor['d'] > 20 * dstar:
                    return goals
                # thetha = self.cur_pos.pose.pose.orientation.z

                orientation_q = self.cur_pos.pose.pose.orientation
                orientation_list = [
                    orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                thetha = yaw

                proportional = Kp * (self.goal_tensor['d'] - dstar)
                integral += Ki * \
                    (self.goal_tensor['d'] - dstar) * self.goal_tensor['dt']
                integral = min(integral, int_max)
                self.vel.linear.x = (proportional + integral)
                self.vel.angular.z = Tp * \
                    atan2(sin(self.goal_tensor['slope'] - thetha),
                          cos(self.goal_tensor['slope'] - thetha))
                self.vel_pub.publish(self.vel)
                self.t = True
                self.rate.sleep()

            goals += 1

    def pi_spiral_mv(self, growth, alpha, dstar, Kp, Ki, Tp, int_max):
        parts = 32
        w = (2*pi)/parts
        proportional = 0
        integral = 0
        goals = 0
        for t in count():
            self.goal_pos.x = (growth*t)*cos(w*t - alpha)
            self.goal_pos.y = (growth*t)*sin(w*t - alpha)
            print(f'goal is ({self.goal_pos.x}, {self.goal_pos.y})')
            self.goal_pub.publish(self.goal_pos)
            self.t = True
            self.rate.sleep()

            while not (self.goal_tensor['d'] < dstar):
                if self.goal_tensor['d'] > 20 * dstar:
                    return goals

                orientation_q = self.cur_pos.pose.pose.orientation
                orientation_list = [
                    orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                thetha = yaw

                proportional = Kp * (self.goal_tensor['d'] - dstar)
                integral += Ki * \
                    (self.goal_tensor['d'] - dstar) * self.goal_tensor['dt']
                integral = min(integral, int_max)
                self.vel.linear.x = (proportional + integral)
                self.vel.angular.z = Tp * \
                    atan2(sin(self.goal_tensor['slope'] - thetha),
                          cos(self.goal_tensor['slope'] - thetha))
                self.vel_pub.publish(self.vel)
                self.t = True
                self.rate.sleep()

            goals += 1

    def shutdown(self):
        print("Shutdown!")
        # stop TurtleBot
        rospy.loginfo("Stop TurtleBot")

        self.stop()

        # makes sure robot receives the stop command prior to shutting down
        rospy.sleep(1)


def tuner(controller):
    data = [x for x in product(range(1, 10), repeat=3)]
    random.shuffle(data)
    m = 0
    mKp, mKi, mKp = 0, 0, 0
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    for Ki, Tp, Kp in data:
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_waffle'
        set_state(state_msg)
        temp = controller.pi_ellipse_mv(5, 5, 0.5, Kp/10, Ki/10, Tp/10)
        if temp > m:
            m = temp
            mKp, mKi, mKp = Kp, Ki, Tp
            print(f'max {m}, Kp:{mKp}, Ki:{mKi}, Tp:{mKp}')

    print(f'final max {m}, Kp:{mKp}, Ki:{mKi}, Tp:{mKp}')


def ellipse_tracking():
    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_waffle'
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state_msg.pose.position.x = 1
    state_msg.pose.position.y = 2
    state_msg.pose.position.z = 0

    state_msg.pose.orientation.z = 0.707
    state_msg.pose.orientation.w = 0.707
    resp = set_state(state_msg)
    print(resp)
    controller.pi_ellipse_mv(1, 3, 0, -pi/2, 0.1, 0.4, 0.1, 0.8, 0.2)

def spiral_tracking():
    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_waffle'
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state_msg.pose.position.x = 1
    state_msg.pose.position.y = 2
    state_msg.pose.position.z = 0

    state_msg.pose.orientation.z = 0.01
    resp = set_state(state_msg)
    print(resp)
    controller.pi_spiral_mv(0.1, pi, 0.25, 0.3, 0.1, 0.7, 0.2)

if __name__ == '__main__':
    try:
        controller = TurtleController()
        rospy.sleep(1)

        # tuner(controller)

        # controller.pi_ellipse_mv2(5, 5, 5, pi/2, 3, 0.5, 0.1, 0.35, 0.2) #circle
        # controller.pi_ellipse_mv2(1, 3, 0, -pi/2, 0.5, 0.5, 0.1, 0.95, 0.2)
        # controller.pi_ellipse_mv2(1, 3, 0, -pi/2, 0.5, 1.3, 0.2, 1.9, 0.4)

        # Call the function for elliptical path tracking
        ellipse_tracking()

        # Call the function for spiral path tracking
        # spiral_tracking()

        print("One round finished")

        # while True:
        # controller.pi_ellipse_mv(5, 5, 0.3, 0.3, 0.1, 0.3)
        # print('One round completed.')

        rospy.spin()

    except Exception as e:
        rospy.loginfo("TurtleController node terminated: " + str(e))
