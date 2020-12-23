#!/usr/bin/env python
import rospy

# cmd_vel Twist data type
from geometry_msgs.msg import Twist
# wheel odometry data type
from nav_msgs.msg import Odometry
# PID input data type
from std_msgs.msg import Header

from gazebo_msgs.srv import GetModelState, GetModelStateRequest

GAZEBO_ODOM='/gazebo/get_model_state'
ODOM_TOPIC='/odom'

odom = Odometry()
header = Header()

header.frame_id='/odom'
model.model_name='cart'

RELAY = None


def callback_odometry(data):
    RELAY.handle_odometry_data(data)


def main():
    global RELAY
    RELAY = Relay()
    RELAY.start()


class Relay(object):
    def __init__(self):
        self.pub_odom = None
        rospy.init_node('odom_pub', anonymous=False)

    def start(self):
        print("Relay node active")
        rospy.Subscriber(GAZEBO_ODOM, GetModelstate, callback_odometry)
        self.pub_odom = rospy.Publisher(ODOM_TOPIC, Odometry)
        rospy.spin()

    def handle_odometry_data(self, data):
        result = data(model)
        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        header.stamp = rospy.get_time()
        odom.header = header
        self.pub_odom.publish(odom)


if __name__ == '__main__':
    main()
