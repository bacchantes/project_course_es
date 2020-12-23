#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <sstream>
#include <iostream>   // std::cout
#include <string>
#include <math.h>

class TwistToAackermann {
    public:
        TwistToAackermann()
        {
        //Topic you want to publish
        pubLeftWheelHinge_ = n_.advertise<std_msgs::Float64>("/cart/joint_lwh_position_controller/command", 1);
        pubRightWheelHinge_ = n_.advertise<std_msgs::Float64>("/cart/joint_rwh_position_controller/command", 1);
        pubLeftWheelRotate_ = n_.advertise<std_msgs::Float64>("/cart/joint_lw_velocity_controller/command", 1);
        pubRightWheelRotate_ = n_.advertise<std_msgs::Float64>("/cart/joint_rw_velocity_controller/command", 1);
        pubLeftBackWheelRotate_ = n_.advertise<std_msgs::Float64>("/cart/joint_lbw_velocity_controller/command", 1);
        pubRightBackWheelRotate_ = n_.advertise<std_msgs::Float64>("/cart/joint_rbw_velocity_controller/command", 1);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/cmd_vel", 1, &TwistToAackermann::callback, this);
        }

        void callback(const geometry_msgs::Twist& msg) {
            // Vehicle wheel base geometry. wheelBase is the distance between front
            // and back axles, track is the distance between left and right wheels.
            double wheelBase = 1.753;
            double track = 1.245;
            double wheelRadius = 0.292;
            double MaxR = 3.81;
            std_msgs::Float64 lwh;
            std_msgs::Float64 rwh;
            std_msgs::Float64 lwr;
            std_msgs::Float64 rwr;
            std_msgs::Float64 lbwr;
            std_msgs::Float64 rbwr;
            double r = 0;


            if (msg.angular.z==0 || msg.linear.x==0) {
                lwh.data = 0;
                rwh.data = 0;
                lwr.data = linearToAngularVelocity(msg.linear.x, wheelRadius);
                rwr.data = linearToAngularVelocity(msg.linear.x, wheelRadius);
                lbwr.data = linearToAngularVelocity(msg.linear.x, wheelRadius);
                rbwr.data = linearToAngularVelocity(msg.linear.x, wheelRadius);
            }
            else {
                // The radius of the turn.
                r = msg.linear.x/msg.angular.z;
                ROS_DEBUG("%f\n", r);
                //if (r<MaxR && r>0) r = MaxR;
                //if (r>-MaxR && r<0) r = -MaxR;

                lwh.data = atan(wheelBase/(r+(track/2)));
                rwh.data = atan(wheelBase/(r+(track/2)));
                lwr.data = linearToAngularVelocity((r-(track/2))*msg.angular.z,wheelRadius);
                rwr.data = linearToAngularVelocity((r+(track/2))*msg.angular.z,wheelRadius);
                lbwr.data = linearToAngularVelocity((r-(track/2))*msg.angular.z,wheelRadius);
                rbwr.data = linearToAngularVelocity((r+(track/2))*msg.angular.z,wheelRadius);    }

            //double R = static_cast<double>(msg.linear.x/msg.angular.z);
            ROS_DEBUG("Test");

            //pubADS_.publish(ads);
            pubLeftWheelHinge_.publish(lwh);
            pubRightWheelHinge_.publish(rwh);
            pubLeftWheelRotate_.publish(lwr);
            pubRightWheelRotate_.publish(rwr);
            pubLeftBackWheelRotate_.publish(lbwr);
            pubRightBackWheelRotate_.publish(rbwr);
        }
    private:
        ros::NodeHandle n_;
        ros::Publisher pubLeftWheelHinge_;
        ros::Publisher pubRightWheelHinge_;
        ros::Publisher pubLeftWheelRotate_;
        ros::Publisher pubRightWheelRotate_;
        ros::Publisher pubLeftBackWheelRotate_;
        ros::Publisher pubRightBackWheelRotate_;
        ros::Subscriber sub_;

    /*
    ** The Gazebo JointVelocityController that spins the front wheels receives
    ** an angular velocity, Radians per second, so we use this method to
    ** convert the linear velocity into angular velocity by dividing it by
    ** the wheel radius, giving radians per second instead of meters per second.
    */
    float linearToAngularVelocity(float linearVelocity, float radius) {
        return linearVelocity/radius;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist_to_ackermann");

    TwistToAackermann SAPObject;
    ros::spin();

    return 0;
}
