/**
 * @file rate_controller.cpp
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

#include "flight_controller.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "simone_msgs/MotorCTRL.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "quadrotor_controller");
    ros::NodeHandle n;

    // Create flight controller object
    FlightController flight_controller(&n);

    // Subscribe to imu and twist messages
    ros::Subscriber sub_imu = n.subscribe("phone_imu", 1000, &FlightController::imuCallback, &flight_controller);
    //ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1000, &FlightController::twistCallback, &flight_controller);
    ros::Subscriber sub_thrust = n.subscribe("command/thrust", 1000, &FlightController::thrustCallback, &flight_controller);
    ros::Subscriber sub_yaw = n.subscribe("command/yawrate", 1000, &FlightController::yawCallback, &flight_controller);
    ros::Subscriber sub_att = n.subscribe("command/attitude", 1000, &FlightController::attitudeCallback, &flight_controller);

    // Set up main loop
    ros::Rate r(50); // 50hz

    ros::spin();

    return true;
}


