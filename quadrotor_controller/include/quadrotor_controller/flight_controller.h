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

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

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

#define PITCH_STAB_KP 4.5
#define PITCH_STAB_KD 0.0 
#define PITCH_STAB_KI 0.0

#define ROLL_STAB_KP 4.5
#define ROLL_STAB_KD 0.0
#define ROLL_STAB_KI 0.0

#define YAW_STAB_KP 10
#define YAW_STAB_KD 0.0
#define YAW_STAB_KI 0.0

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
    Vect3F getOutput(Vect3F sensor_orientation
private:
    MiniPID m_pitch_att_pid;
    MiniPID m_roll_att_pid;
    MiniPID m_yaw_att_pid;
};

class FlightController
{
public:
    FlightController();
    ~FlightController();

    // Subscriber callback functions
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
private:
    double m_thrust;
    Motors m_motors;
    FlightMode m_flight_mode;
    RateController m_rate_controller;
    AttitudeController m_att_controller;

    // Private methods
    void applyThrustAdjustments(Vect3F thrust_adjustments);
};

#endif // FLIGHT_CONTROLLER_H
