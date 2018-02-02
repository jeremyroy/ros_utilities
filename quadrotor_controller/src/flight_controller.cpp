/**
 * @file rate_controller.cpp
 * @brief cpp version of a quadcopter rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

//////////////
// Includes //
//////////////

#include "flight_controller.h"
#include "quadrotor_controller/MotorCTRL.h"

#include <cmath>

//#include <ros/console.h>

/////////////////////////////
// Methods - RateController//
/////////////////////////////

RateController::RateController()
    : m_pitch_rate_pid(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD),
      m_roll_rate_pid(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD),
      m_yaw_rate_pid(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD)
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


/////////////////////////////////
// Methods - AttitudeController//
/////////////////////////////////

AttitudeController::AttitudeController()
    : m_pitch_att_pid(PITCH_ATT_KP, PITCH_ATT_KI, PITCH_ATT_KD),
      m_roll_att_pid(ROLL_ATT_KP, ROLL_ATT_KI, ROLL_ATT_KD),
      m_yaw_att_pid(YAW_ATT_KP, YAW_ATT_KI, YAW_ATT_KD)
{
    // Empty constructor
}


AttitudeController::~AttitudeController()
{
    // Empty destructor
}


void AttitudeController::setDesiredAtt(Vect3F orientation)
{
    m_pitch_att_pid.setSetpoint(orientation.x);
    m_roll_att_pid.setSetpoint(orientation.y);
    m_yaw_att_pid.setSetpoint(orientation.z);
}


Vect3F AttitudeController::getOutput(Vect3F sensor_orientation)
{
    double sensor_roll_att(sensor_orientation.y);
    double sensor_pitch_att(sensor_orientation.x);
    double sensor_yaw_att(sensor_orientation.z);

    // TODO: verify that the measure and desired reference frames are alligned
    // 
    // Run controller 
    // Assume both angular velocities are in radians per second
    //
    // Calculate andular rate adjustments (in % of full scale)
    double roll_rate_adj(0.0);
    double pitch_rate_adj(0.0);
    double yaw_rate_adj(0.0);
    
    roll_rate_adj  = m_roll_att_pid.getOutput(sensor_roll_att);
    pitch_rate_adj = m_pitch_att_pid.getOutput(sensor_pitch_att);
    yaw_rate_adj   = m_yaw_att_pid.getOutput(sensor_yaw_att);

    // Format and return angular rate adjustments
    Vect3F rate_adj;
    rate_adj.x = roll_rate_adj;
    rate_adj.y = pitch_rate_adj;
    rate_adj.z = yaw_rate_adj;

    return rate_adj;
}


///////////////////////////////
// Methods - FlightController//
///////////////////////////////

FlightController::FlightController(const ros::Publisher *publisher)
    : m_thrust(0.0),
      m_flight_mode(DISABLE),
      m_motor_publisher(publisher)
{
   // Empty constructor 
}


FlightController::~FlightController()
{
    // Empty destructor
}


void FlightController::applyThrustAdjustments(Vect3F thrust_adjustments)
{
    double roll_thrust_adj, pitch_thrust_adj, yaw_thrust_adj;
    double motor1_thrust, motor2_thrust, motor3_thrust, motor4_thrust;

    quadrotor_controller::MotorCTRL motor_ctrl_msg;
    
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

    // Publish motor control message
    motor_ctrl_msg.m1 = motor1_thrust;
    motor_ctrl_msg.m2 = motor2_thrust;
    motor_ctrl_msg.m3 = motor3_thrust;
    motor_ctrl_msg.m4 = motor4_thrust;

    m_motor_publisher->publish(motor_ctrl_msg);

    /*// Send new thrust to motors
    m_motors.setMotorThrust(MOTOR_1, motor1_thrust);
    m_motors.setMotorThrust(MOTOR_2, motor2_thrust);
    m_motors.setMotorThrust(MOTOR_3, motor3_thrust);
    m_motors.setMotorThrust(MOTOR_4, motor4_thrust);
    */
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

void FlightController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Organize incomming sensor data in less verbose structures
    Vect3F rates(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    
    Quat quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Vect3F orientation = quat2Euler(quat);
    
    // Run controller every time new sensor data is received.
    switch(m_flight_mode)
    {
        case ACRO:
        {
            // Calculate acro thrust adjustments
            Vect3F thrust_adjustments;
            thrust_adjustments = m_rate_controller.getOutput(rates);
            // Send thrust adjustments to motors
            this->applyThrustAdjustments(thrust_adjustments);
            break;
        }

        case STABL:
        {
            // Calculate stabalize angular velocity adjustments
            Vect3F rate_adjustments;
            rate_adjustments = m_att_controller.getOutput(orientation);
            // Calculate stabalize thrust adjustments
            Vect3F thrust_adjustments;
            thrust_adjustments = m_rate_controller.getOutput(rate_adjustments);
            // Send thrust adjustments to motors
            this->applyThrustAdjustments(thrust_adjustments);
            break;
        }
        
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
    if (msg->linear.y == 0)
    {
        m_flight_mode = DISABLE;
    }
    else if (msg->linear.y == 1)
    {
        m_flight_mode = ACRO;
    }
    else if (msg->linear.y == 2)
    {
        m_flight_mode = STABL;
    }
    else if (msg->linear.y == 3)
    {
        m_flight_mode = AUTO;
    }

    // Organize incomming angular control data in less verbose structure
    Vect3F commanded_values(msg->angular.x, msg->angular.y, msg->angular.z); 
    
    // Update controller's angular reference points based on flight mode
    switch(m_flight_mode)
    {
        case ACRO:
            m_rate_controller.setDesiredRates(commanded_values);
            break;
        
        case STABL:
            //TODO: might need to set desired yaw angle as current_yaw + input value
            m_att_controller.setDesiredAtt(commanded_values);
            break;
        
        case AUTO:
            //TODO
        
        default:
            m_thrust = 0.0;
            m_motors.killAll();
    }

}

