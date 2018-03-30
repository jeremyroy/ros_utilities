/**
 * @file quadrotor_leveler_node.cpp
 * @brief Sends modified attitude commands to a quadrotor to counteracting IMU bias.
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

//////////////
// Includes //
//////////////

#include <ros/ros.h>
#include <ros/console.h>

#include "sensor_msgs/Imu.h"
#include "simone_msgs/MotorCTRL.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "hector_uav_msgs/AttitudeCommand.h"

#include <chrono>
#include <fstream>

std::ofstream outfile("time_stamp.txt");

/////////////
// Defines //
/////////////

#define CAL_NUMBER_OF_READINGS 200

///////////
// Types //
///////////

typedef struct Vect3F
{
    Vect3F() {}
    Vect3F(double a, double b, double c) : x(a), y(b), z(c) {}

    double x;
    double y;
    double z;
} Vect3F;

typedef struct Quat
{
    Quat() {}
    Quat(double a, double b, double c, double d) : w(a), x(b), y(c), z(d) {}

    double w;
    double x;
    double y;
    double z;
} Quat;

//////////////////////
// Global Variables //
//////////////////////

bool g_calibrated = true;
bool g_calibrating = false;
double g_roll_bias = 0.0;
double g_pitch_bias = 0.0;

ros::Publisher g_pub_ajusted_attitude;

///////////////
// Functions //
///////////////

bool recal(std_srvs::Empty::Request  &req,
           std_srvs::Empty::Response &res)
{
    g_calibrated = false;
}

// This function is from wikipedia.
Vect3F quat2Euler(Quat q)
{
    Vect3F rpy;

    // Yaw (z-axis rotation)
    double siny = +2.0 * (-q.y * q.z + q.w * q.x);
    double cosy = -q.z * q.z + q.y * q.y - q.x * q.x + q.w * q.w; 
    rpy.z = -std::atan2(siny, -cosy);

    // Roll (x-axis rotation)
    double sinr = +2.0 * (q.x * q.y + q.w * q.z);
    rpy.x = -std::asin(sinr);

    // Pitch (y-axis rotation)
    double sinp = +2.0 * (-q.x * q.z + q.w * q.y);
    double cosp = - q.z * q.z - q.y * q.y + q.x * q.x + q.w * q.w;  
    rpy.y = -std::atan2(sinp, cosp);

    return rpy;
}

// Calibrates attitude commands by counteracting IMU orientation bias
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    static int sum_count = 0;
    if (!g_calibrated)
    {
        /* Check if calibration is starting */
        if (sum_count == 0)
        {
            ROS_INFO("Starting Calibration");

            g_calibrating = true;

            // Reset biases
            g_roll_bias = 0.0;
            g_pitch_bias = 0.0;
        }

        /* Calibrate */
        // Translate orientation to euler angles
        Quat quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Vect3F orientation = quat2Euler(quat);

        // Sum biases
        g_roll_bias += orientation.y;
        g_pitch_bias += orientation.x;
        ++sum_count;

        /* Check if calibration is finished */
        if (sum_count >= CAL_NUMBER_OF_READINGS)
        {
            // Adjust biases to average
            g_roll_bias /= sum_count;
            g_pitch_bias /= sum_count;
            // Terminate calibration
            sum_count = 0;
            g_calibrated = true;
            g_calibrating = false;

            ROS_INFO("Finished Calibration");
        }
    }
}

void attitudeCallback(const hector_uav_msgs::AttitudeCommand::ConstPtr& msg)
{
    if (g_calibrated && !g_calibrating)
    {
        // Adjust scale of commanded attitude values to +-45Deg
        double roll_cmd = msg->roll * 1.5; 
        double pitch_cmd = msg->pitch * 1.5;

        // Adjust commanded attitude
        roll_cmd += g_roll_bias;
        pitch_cmd += g_pitch_bias;

        // Format adjusted attitude message
        hector_uav_msgs::AttitudeCommand new_att_commands;
        new_att_commands.roll = roll_cmd;
        new_att_commands.pitch = pitch_cmd;

        // Publish formatted message
        g_pub_ajusted_attitude.publish(new_att_commands);
        
        // Latency test: If it's the first time roll is greater than 0.6, record current time
        static bool time_logged = false;
        if ((roll_cmd >= 0.785398185253) && (time_logged == false))
        {
            time_logged = true;
            std::chrono::time_point<std::chrono::system_clock> now = 
                    std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            outfile << millis << std::endl;
            outfile.close();
        }
    }
}

//////////
// Main //
//////////

int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "quadrotor_leveler");
    ros::NodeHandle n;
    
    // Advertise leveled attitude
    g_pub_ajusted_attitude = n.advertise<hector_uav_msgs::AttitudeCommand>("command/attitude_adjusted", 1000);
    
    // Subscribe to imu and attitude topics
    ros::Subscriber sub_imu = n.subscribe("phone_imu", 1000, imuCallback);
    ros::Subscriber sub_att = n.subscribe("command/attitude", 1000, attitudeCallback);

    // Provide service to re-calibrate
    ros::ServiceServer service = n.advertiseService("recalibrate_attitude", recal);

    // Set up main loop
    ros::Rate r(50); // 50hz

    ros::spin();

    return true;
}


