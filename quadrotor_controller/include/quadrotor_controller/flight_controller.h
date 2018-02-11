/**
 * @file rate_controller.h
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

//////////////
// Includes //
//////////////

#include <array>

#include "motors_api.h"

#include "MiniPID.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

#include "hector_uav_msgs/YawrateCommand.h"
#include "hector_uav_msgs/ThrustCommand.h"
#include "hector_uav_msgs/AttitudeCommand.h"

/////////////
// Defines //
/////////////

#define PITCH_RATE_KP 0.7
#define PITCH_RATE_KD 0.0 
#define PITCH_RATE_KI 0.001

#define ROLL_RATE_KP 0.7
#define ROLL_RATE_KD 0.0
#define ROLL_RATE_KI 0.001

#define YAW_RATE_KP 2.5
#define YAW_RATE_KD 0.0
#define YAW_RATE_KI 0.001

#define PITCH_ATT_KP 4.5
#define PITCH_ATT_KD 0.0 
#define PITCH_ATT_KI 0.0

#define ROLL_ATT_KP 4.5
#define ROLL_ATT_KD 0.0
#define ROLL_ATT_KI 0.0

#define YAW_ATT_KP 10
#define YAW_ATT_KD 0.0
#define YAW_ATT_KI 0.0

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

typedef struct RotMatrix
{
    double R11, R12, R13;
    double R21, R22, R23;
    double R31, R32, R33;
} RotMatrix;

typedef enum FlightMode
{
    DISABLE = 0,
    ACRO    = 1,
    STABL   = 2,
    AUTO    = 3
} FlightMode;

/////////////
// Classes //
/////////////

class RateController
{
public:
    RateController();
    ~RateController();
    
    void setDesiredRates(Vect3F rates);
    Vect3F getOutput(Vect3F sensor_rates);
private:
    MiniPID m_pitch_rate_pid;
    MiniPID m_roll_rate_pid;
    MiniPID m_yaw_rate_pid;
};

class AttitudeController
{
public:
    AttitudeController();
    ~AttitudeController();

    void setDesiredAtt(Vect3F orientation);
    Vect3F getOutput(Vect3F sensor_orientation);
private:
    MiniPID m_pitch_att_pid;
    MiniPID m_roll_att_pid;
    MiniPID m_yaw_att_pid;
};

class FlightController
{
public:
    FlightController(const ros::Publisher *publisher);
    ~FlightController();

    // Subscriber callback functions
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    //void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void thrustCallback(const hector_uav_msgs::ThrustCommand::ConstPtr& msg);
    void yawCallback(const hector_uav_msgs::YawrateCommand::ConstPtr& msg);
    void attitudeCallback(const hector_uav_msgs::AttitudeCommand::ConstPtr& msg);
private:
    // State attributes
    double m_thrust;
    double m_pitch;
    double m_roll;
    double m_yaw;
    FlightMode m_flight_mode;

    // Motors and controllers
    //Motors m_motors;
    RateController m_rate_controller;
    AttitudeController m_att_controller;

    const ros::Publisher *m_motor_publisher;

    // Private methods
    void applyThrustAdjustments(Vect3F thrust_adjustments);
};

#endif // FLIGHT_CONTROLLER_H
