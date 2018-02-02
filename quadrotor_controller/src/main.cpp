/**
 * @file rate_controller.cpp
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

#include "flight_controller.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "quadrotor_controller/MotorCTRL.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "quadrotor_controller");
    ros::NodeHandle n;

    // Create MotorCTRL puslisher
    ros::Publisher pub_motors = n.advertise<quadrotor_controller::MotorCTRL>("motor_ctrl", 1000);

    // Create flight controller object
    FlightController flight_controller(&pub_motors);

    // Subscribe to imu and twist messages
    ros::Subscriber sub_imu = n.subscribe("phone_imu", 1000, &FlightController::imuCallback, &flight_controller);
    ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1000, &FlightController::twistCallback, &flight_controller);

    // Set up main loop
    ros::Rate r(50); // 50hz

    ros::spin();

    return true;
}


