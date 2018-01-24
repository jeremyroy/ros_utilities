/**
 * @file rate_controller.h
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

//////////////
// Includes //
//////////////

#include "motors_api.h"

#include "MiniPID.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

/////////////
// Defines //
/////////////

#define PITCH_RATE_KP 2.5
#define PITCH_RATE_KD 0.0 
#define PITCH_RATE_KI 0.01

#define ROLL_RATE_KP 2.5
#define ROLL_RATE_KD 0.0
#define ROLL_RATE_KI 0.01

#define YAW_RATE_KP 2.5
#define YAW_RATE_KD 0.0
#define YAW_RATE_KI 0.01

/////////////
// Classes //
/////////////

class FlightController
{
public:
    FlightController();
    ~FlightController();
private:
    MiniPID m_pitch_rate_pid;
    MiniPID m_roll_rate_pid;
    MiniPID m_yaw_rate_pid;
    double m_thrust;
    Motors m_motor;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
};
