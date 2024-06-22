/*
 * ROS Node for Inverse Kinematics of a Differential Drive Robot
 * 
 * This node subscribes to velocity commands (/cmd_vel), computes the target velocities
 * for the left and right wheels using inverse kinematics, and publishes these velocities
 * to the respective controllers (/robot/joint1_velocity_controller/command and
 * /robot/joint2_velocity_controller/command). It also subscribes to odometry messages
 * (/wheel/odom) to update the orientation (theta) of the robot.
 * 
 * Maintainer: chehabi.mohammed@gmail.com
 * License: BSD
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

// Global variables for motor rate and buffer size
int motor_rate;
int buffer_size;

class DifferentialDriveRobot {
public:
    DifferentialDriveRobot() {
        // Initialize ROS node and create publishers for joint velocities
        ros::NodeHandle nh;
        joint1_vel_pub = nh.advertise<std_msgs::Float64>("robot/joint1_velocity_controller/command", buffer_size);
        joint2_vel_pub = nh.advertise<std_msgs::Float64>("robot/joint2_velocity_controller/command", buffer_size);

        // Subscribe to /wheel/odom to get the orientation (theta)
        theta_sub = nh.subscribe("/wheel/odom", 100, &DifferentialDriveRobot::thetaCallback, this);

        // Subscribe to /cmd_vel to receive velocity commands
        cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &DifferentialDriveRobot::inverseKinematicCallback, this);

        // Initialize parameters from ROS parameter server
        nh.param<double>("wheel_separation", wheel_separation, 0.138);
        nh.param<double>("wheel_radius", wheel_radius, 0.032);
        nh.param<int>("motor_rate", motor_rate, 100);
        nh.param<int>("buffer_size", buffer_size, 10);

        // Initialize variables
        v = 0.0;
        w = 0.0;
        target_velocity_left = 0.0;
        target_velocity_right = 0.0;
    }

    // Callback function to handle odometry messages and extract orientation (theta)
    void thetaCallback(const nav_msgs::Odometry::ConstPtr& theta_msg) {
        theta = theta_msg->pose.pose.orientation;
    }

    // Callback function to handle velocity commands and compute wheel velocities
    void inverseKinematicCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
        // Extract yaw (theta) from quaternion
        theta_ = tf::getYaw(theta);

        // Extract linear and angular velocities from the message
        v = cmd_vel_msg->linear.x;
        w = cmd_vel_msg->angular.z; 

        // Calculate the target velocities for left and right wheels in rad/s
        target_velocity_left = (v - (w * wheel_separation / 2.0)) / wheel_radius;
        target_velocity_right = (v + (w * wheel_separation / 2.0)) / wheel_radius;

        // Create and publish the wheel velocities as std_msgs::Float64 messages
        std_msgs::Float64 joint1_msg;
        std_msgs::Float64 joint2_msg;
        joint1_msg.data = target_velocity_left;
        joint2_msg.data = target_velocity_right;

        joint1_vel_pub.publish(joint1_msg);
        joint2_vel_pub.publish(joint2_msg);
    }

private:
    ros::Publisher joint1_vel_pub;  // Publisher for left wheel velocity
    ros::Publisher joint2_vel_pub;  // Publisher for right wheel velocity
    ros::Subscriber cmd_vel_sub;    // Subscriber for velocity commands
    ros::Subscriber theta_sub;      // Subscriber for orientation updates

    geometry_msgs::Quaternion theta; // Robot orientation in quaternion
    double wheel_radius;             // Radius of the wheels
    double wheel_separation;         // Distance between the wheels
    double theta_;                   // Orientation in radians (yaw)
    double v;                        // Linear velocity
    double w;                        // Angular velocity
    double target_velocity_left;     // Target velocity for left wheel
    double target_velocity_right;    // Target velocity for right wheel
};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "inverse_kinematic_converter");

    // Create DifferentialDriveRobot object
    DifferentialDriveRobot robot;

    // Set the control loop rate
    ros::Rate rate(motor_rate);

    // Main control loop
    while (ros::ok()) {
        ros::spinOnce(); // Process incoming messages
        rate.sleep();    // Sleep to maintain the loop rate
    }

    return 0;
}
