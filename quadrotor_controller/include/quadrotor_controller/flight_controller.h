/**
 * @file rate_controller.h
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

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

#define PITCH_RATE_KP 2.5
#define PITCH_RATE_KD 0.0 
#define PITCH_RATE_KI 0.01

#define ROLL_RATE_KP 2.5
#define ROLL_RATE_KD 0.0
#define ROLL_RATE_KI 0.01

#define YAW_RATE_KP 2.5
#define YAW_RATE_KD 0.0
#define YAW_RATE_KI 0.01

///////////
// Types //
///////////

typedef struct Vect3F
{
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

class RateController()
{
public:
    RateController();
    ~RateController();
    
    void setDesiredRates(Vect3F rates);
    Vect3F getOutput(Vect3F sensor_rates)
private:
    MiniPID m_pitch_rate_pid;
    MiniPID m_roll_rate_pid;
    MiniPID m_yaw_rate_pid;
};

class FlightController
{
public:
    FlightController();
    ~FlightController();

private:
    double m_thrust;
    Motors m_motor;
    FlightMode m_flight_mode;

    // Private methods
    void applyThrustAdjustments(Vect3F thrust_adjustments);

    // Subscriber callback functions
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
};
