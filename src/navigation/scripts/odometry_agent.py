#!/usr/bin/env python
import rospy
import gazebo_ros
# wheel odometry data type
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


GAZEBO_TOPIC='/gazebo/model_states'
ODOM_TOPIC='/odom'



RELAY = None

def callback_odometry(data):
    RELAY.handle_odometry_data(data)

def main():
    global RELAY
    RELAY = Relay()
    RELAY.start()

class Relay(object):
    def __init__(self):
        self.odom_pub = None
        self.odom = Odometry()
        rospy.init_node('odom_pub', anonymous=False)

    def start(self):
        rospy.Subscriber(GAZEBO_TOPIC, ModelStates, callback_odometry)
        self.odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry)
        rospy.spin()

    def handle_odometry_data(self, data):
        self.odom.pose.pose = data.pose
        self.odom.twist.twist = data.twist
        #odom = data
        self.odom_pub.publish(self.odom)


if __name__ == '__main__':
    main()
