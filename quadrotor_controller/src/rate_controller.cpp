/**
 * @brief cpp version of a quadcopter flight controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

#include "motors_api.h"

#include "MiniPID.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

#define PITCH_RATE_KP 0.7
#define PITCH_RATE_KD 
#define PITCH_RATE_KI 1

#define ROLL_RATE_KP 0.7
#define ROLL_RATE_KD 
#define ROLL_RATE_KI 1

#define YAW_RATE_KP 0.7
#define YAW_RATE_KD 
#define YAW_RATE_KI 1

typedef struct Vect3f
{
    double x;
    double y;
    double z;
} Vect3f;


// Global variables
MiniPID g_pitch_rate_pid(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD);
MiniPID g_roll_rate_pid(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD);
MiniPID g_yaw_rate_pid(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD);

double g_thrust(0.0); // Thrust in % of full scale

Motors g_motors;

// Functions

void run_controller(const sensor_msgs::Imu::ConstPtr& msg)
{
    double sensor_roll_rate(msg->angular_velocity.x);
    double sensor_pitch_rate(msg->angular_velocity.y);
    double sensor_yaw_rate(msg->angular_velocity.z);

    // TODO: verify that the measure and desired reference frames are alligned
    // 
    // Run controller 
    // For now, only consider angular velocity (it's directly measurable)
    // Assume both angular velocities are in degrees per second
    //
    // Calculate thrust adjustments (in % of full scale)
    double roll_thrust_adj(0.0);
    double pitch_thrust_adj(0.0);
    double yaw_thrust_adj(0.0);
    
    roll_thrust_adj  = g_roll_rate_pid.getOutput(sensor_roll_rate);
    pitch_thrust_adj = g_pitch_rate_pid.getOutput(sensor_pitch_rate);
    yaw_thrust_adj   = g_yaw_rate_pid.getOutput(sensor_yaw_rate);

    // Send thrust adjustments to motors
    double motor1_thrust(0);
    double motor2_thrust(0);
    double motor3_thrust(0);
    double motor4_thrust(0);

    motor1_thrust = g_thrust + roll_thrust_adj - pitch_thrust_adj + yaw_thrust_adj;
    motor2_thrust = g_thrust - roll_thrust_adj + pitch_thrust_adj + yaw_thrust_adj;
    motor3_thrust = g_thrust - roll_thrust_adj - pitch_thrust_adj - yaw_thrust_adj;
    motor4_thrust = g_thrust + roll_thrust_adj + pitch_thrust_adj - yaw_thrust_adj;

    g_motors.setMotorThrust(MOTOR_1, motor1_thrust);
    g_motors.setMotorThrust(MOTOR_2, motor2_thrust);
    g_motors.setMotorThrust(MOTOR_3, motor3_thrust);
    g_motors.setMotorThrust(MOTOR_4, motor4_thrust);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Run controller every time new sensor data is received.
    run_controller(msg);
}

void twistCallback(const geometry_msgs::Twist::constPtr& msg)
{
    g_roll_rate_pid.setSetpoint(msg->angular.x);
    g_pitch_rate_pid.setSetpoint(msg->angular.y);
    g_yaw_rate_pid.setSetpoint(msg->angular.z);

    g_thrust = msg->linear.z;
}

int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "quadrotor_controller");
    ros::NodeHandle n;

    ros::Subscriber sub_imu = n.subscribe("phone_imu", 1000, imuCallback);
    ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1000, twistCallback);

    ros::Rate r(50); // 50hz

    ros::spin();

    return true;
}


