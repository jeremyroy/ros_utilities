/**
 * @file rate_controller.cpp
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

//////////////
// Includes //
//////////////

#include "include/quadrotor_controller/rate_controller.h"

/////////////////////////////
// Methods - RateController//
/////////////////////////////

RateController::RateController()
    : m_pitch_rate_pid(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD),
      m_roll_rate_pid(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD),
      m_yaw_rate_pid(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD),
{
    // Empty constructor
}


RateController::~RateController()
{
    // Empty destructor
}


void RateController::setDesiredRates(Vect3F rates)
{
    m_pitch_rate_pid.setSetpoint(rates.x);
    m_roll_rate_pid.setSetpoint(rates.y);
    m_yaw_rate_pid.setSetpoint(rates.z);
}

// Returns thrust adjustments in RPY
Vect3F RateController::getOutput(Vect3F sensor_rates)
{
    double sensor_roll_rate(sensor_rates.y);
    double sensor_pitch_rate(sensor_rates.x);
    double sensor_yaw_rate(sensor_rates.z);

    // TODO: verify that the measure and desired reference frames are alligned
    // 
    // Run controller 
    // Assume both angular velocities are in radians per second
    //
    // Calculate thrust adjustments (in % of full scale)
    double roll_thrust_adj(0.0);
    double pitch_thrust_adj(0.0);
    double yaw_thrust_adj(0.0);
    
    roll_thrust_adj  = m_roll_rate_pid.getOutput(sensor_roll_rate);
    pitch_thrust_adj = m_pitch_rate_pid.getOutput(sensor_pitch_rate);
    yaw_thrust_adj   = m_yaw_rate_pid.getOutput(sensor_yaw_rate);

    // Format and return thrust adjustments
    Vect3F thrust_adj;
    thrust_adj.x = roll_thrust_adj;
    thrust_adj.y = pitch_thrust_adj;
    thrust_adj.z = yaw_thrust_adj;

    return thrust_adj;
}


///////////////////////////////
// Methods - FlightController//
///////////////////////////////
FlightController::FlightController()
    : m_thrust(0.0),
      m_flight_mode(DISABLE)
{
   // Empty constructor 
}


FlightController::~FlightController()
{
    // Empty destructor
}


std::array<double,4> rate_controller(const sensor_msgs::Imu::ConstPtr& msg)
{

}

void FlightController::applyThrustAdjustments(Vect3F thrust_adjustments)
{
    double roll_thrust_adj, pitch_thrust_adj, yaw_thrust_adj;
    double motor1_thrust, motor2_thrust, motor3_thrust, motor4_thrust;
    
    // Re-assign input vector to make it's values more comprehensive
    roll_thrust_adj  = thrust_adjustments.x;
    pitch_thrust_adj = thrust_adjustments.y;
    yaw_thrust_adj   = thrust_adjustments.z;
    
    // Calculate new thrusts

    // motor1_thrust = m_thrust + roll_thrust_adj - pitch_thrust_adj + yaw_thrust_adj;
    // motor2_thrust = m_thrust - roll_thrust_adj + pitch_thrust_adj + yaw_thrust_adj;
    // motor3_thrust = m_thrust - roll_thrust_adj - pitch_thrust_adj - yaw_thrust_adj;
    // motor4_thrust = m_thrust + roll_thrust_adj + pitch_thrust_adj - yaw_thrust_adj;

    motor1_thrust = m_thrust - roll_thrust_adj + pitch_thrust_adj + yaw_thrust_adj;
    motor2_thrust = m_thrust + roll_thrust_adj - pitch_thrust_adj + yaw_thrust_adj;
    motor3_thrust = m_thrust + roll_thrust_adj + pitch_thrust_adj - yaw_thrust_adj;
    motor4_thrust = m_thrust - roll_thrust_adj - pitch_thrust_adj - yaw_thrust_adj;

    // Send new thrust to motors
    m_motors.setMotorThrust(MOTOR_1, motor1_thrust);
    m_motors.setMotorThrust(MOTOR_2, motor2_thrust);
    m_motors.setMotorThrust(MOTOR_3, motor3_thrust);
    m_motors.setMotorThrust(MOTOR_4, motor4_thrust);
}

void FlightController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Organize incomming sensor data in less verbose structures
    Vect3F rates(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    
    // Run controller every time new sensor data is received.
    run_controller(msg);
    switch(m_flight_mode)
    {
        case ACRO:
            // Calculate acro thrust adjustments
            Vect3F thrust_adjustments;
            thrust_adjustments = m_rate_controller.getOutput(rates);
            // Send thrust adjustments to motors
            this->applyThrustAdjustments(thrust_adjustments);

        case STABL:
            //TODO
        
        case AUTO:
            //TODO
        
        default:
            m_motors.killAll(); // Should be redundant
    }
}

void FlightController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Global thrust based on Twist message's vertical linear velocity
    // This controller does not attempt to set the quadrotor's linear or
    // angular velocities to the commanded values.  Rather, the controller
    // uses the commanded values as directional commands.
    m_thrust = msg->linear.z;

    // Linear velocity along y determines flight mode
    if (msg->linear.y == 0 || msg->linear.x == 1 || msg->linear.z == 2 || msg->linear.z == 3)
    {
        m_flight_mode = msg->linear.y;
    }

    // Organize incomming angular control data in less verbose structure
    Vect3F commanded_values(msg->angular.x, msg->angular.y, msg->angular.z); 
    
    // Update controller's angular reference points based on flight mode
    switch(m_flight_mode)
    {
        case ACRO:
            m_rate_controller.setDesiredRates(commanded_values);
        
        case STABL:
            //TODO
        
        case AUTO:
            //TODO
        
        default:
            m_thrust = 0.0;
            m_motors.killAll();
    }

}

