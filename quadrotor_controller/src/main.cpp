/**
 * @file rate_controller.cpp
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

#include "include/quadrotor_controller/rate_controller.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "quadrotor_controller");
    ros::NodeHandle n;

    FlightController flight_controller;

    ros::Subscriber sub_imu = n.subscribe("phone_imu", 1000, &FlightController::imuCallback, &flight_controller);
    ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1000, &FlightController::twistCallback, &flight_controller);

    ros::Rate r(50); // 50hz

    ros::spin();

    return true;
}


