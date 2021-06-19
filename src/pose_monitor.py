#!/usr/bin/env python3

import rospy

# Import the Odometry message
from nav_msgs.msg import Odometry

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# Import the Twist message
from geometry_msgs.msg import Twist

# TF allows to perform transformations between different coordinate frames
import tf

# For getting robot's ground truth from Gazebo
from gazebo_msgs.srv import GetModelState


class PoseMonitor():

    def __init__(self):
        # Initiate a named node
        rospy.init_node('pose_monitor', anonymous=True)

        self.t = False
        self.path = Path()
        self.cur_pos = Odometry()

        # Subscribe to topic /odom published by the robot base
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.callback_odometry)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.rate = rospy.Rate(1)

        # subscribe to a service server, provided by the gazebo package to get
        # information about the state of the models present in the simulation

        print("Wait for service ....")
        rospy.wait_for_service("gazebo/get_model_state")

        print(" ... Got it!")

        self.get_ground_truth = rospy.ServiceProxy("/gazebo/get_model_state",
                                                   GetModelState)

    def callback_odometry(self, msg):
        self.cur_pos = msg
        if self.t:

            self.path.header = msg.header
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
            print("Position: (%5.2f, %5.2f, %5.2f)" % (msg.pose.pose.position.x,
                                                       msg.pose.pose.position.y, msg.pose.pose.position.z))
            self.quaternion_to_euler(msg)
            print("Linear twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.linear.x,
                                                           msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            print("Angular twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.angular.x,
                                                            msg.twist.twist.angular.y, msg.twist.twist.angular.z))
            self.t = False

            print("Ground Truth: ", self.get_ground_truth(
                "turtlebot3_waffle", "base_footprint"))

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        print("Roll: %5.2f Pitch: %5.2f Yaw: %5.2f" % (roll, pitch, yaw))


if __name__ == '__main__':
    try:
        monitor = PoseMonitor()

        # keeping doing until ctrl+c
        while not rospy.is_shutdown():
            monitor.t = True
            monitor.rate.sleep()

    except:
        rospy.loginfo("move_robot node terminated")
