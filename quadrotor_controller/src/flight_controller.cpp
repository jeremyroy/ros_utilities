/**
 * @file rate_controller.cpp
 * @brief cpp version of a quadrotor rate controller
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */

//////////////
// Includes //
//////////////

#include "flight_controller.h"
#include "simone_msgs/MotorCTRL.h"

#include <ros/console.h>

#include <cmath>
#include <sstream>

//#include <ros/console.h>

/////////////////////////////
// Methods - RateController//
/////////////////////////////

RateController::RateController(Vect3F roll, Vect3F pitch, Vect3F yaw)
    : m_roll_rate_pid(roll.x, roll.y, roll.z),
      m_pitch_rate_pid(pitch.x, pitch.y, pitch.z),
      m_yaw_rate_pid(yaw.x, yaw.y, yaw.z)
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

void RateController::reset(Vect3F roll, Vect3F pitch, Vect3F yaw)
{
    // Update PID values
    m_roll_rate_pid.setPID(roll.x, roll.y, roll.z);
    m_pitch_rate_pid.setPID(pitch.x, pitch.y, pitch.z);
    m_yaw_rate_pid.setPID(yaw.x, yaw.y, yaw.z);

    // Reset PIDs
    m_roll_rate_pid.reset();
    m_pitch_rate_pid.reset();
    m_yaw_rate_pid.reset();
}


/////////////////////////////////
// Methods - AttitudeController//
/////////////////////////////////

AttitudeController::AttitudeController(Vect3F roll, Vect3F pitch, Vect3F yaw)
    : m_roll_att_pid(roll.x, roll.y, roll.z),
      m_pitch_att_pid(pitch.x, pitch.y, pitch.z),
      m_yaw_att_pid(yaw.x, yaw.y, yaw.z)
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

void AttitudeController::reset(Vect3F roll, Vect3F pitch, Vect3F yaw)
{
    // Update PID values
    m_roll_att_pid.setPID(roll.x, roll.y, roll.z);
    m_pitch_att_pid.setPID(pitch.x, pitch.y, pitch.z);
    m_yaw_att_pid.setPID(yaw.x, yaw.y, yaw.z);

    // Reset PIDs
    m_roll_att_pid.reset();
    m_pitch_att_pid.reset();
    m_yaw_att_pid.reset();
}


///////////////////////////////
// Methods - FlightController//
///////////////////////////////

FlightController::FlightController(ros::NodeHandle *nh)
    : m_thrust(0.0),
      m_flight_mode(DISABLE),
      m_node(nh)
{
    // Create MotorCTRL puslisher
    m_motor_publisher = m_node->advertise<simone_msgs::MotorCTRL>("motor_ctrl", 1000);

    // Load parameters
    this->loadParameters();

    // Initialize the rate and attitude controllers
    m_rate_controller = new RateController(m_roll_rate_pid_terms, 
            m_pitch_rate_pid_terms, m_yaw_rate_pid_terms);
    m_att_controller = new AttitudeController(m_roll_att_pid_terms, 
            m_pitch_att_pid_terms, m_yaw_att_pid_terms);

    // Set up dynamic reconfigure callback
    m_dr_cb = boost::bind(&FlightController::dynamicReconfigureCallback, this, _1, _2);
    m_dr_server.setCallback(m_dr_cb);
}


FlightController::~FlightController()
{
    // Empty destructor
}

void FlightController::loadParameters(void)
{
    // Load the Roll Rate PID parameters
    m_node->param("rate_roll_KP", m_roll_rate_pid_terms.x, DEFAULT_ROLL_RATE_KP);
    m_node->param("rate_roll_KI", m_roll_rate_pid_terms.y, DEFAULT_ROLL_RATE_KI);
    m_node->param("rate_roll_KD", m_roll_rate_pid_terms.z, DEFAULT_ROLL_RATE_KD);

    // Load the Pitch Rate PID parameters
    m_node->param("rate_pitch_KP", m_pitch_rate_pid_terms.x, DEFAULT_PITCH_RATE_KP);
    m_node->param("rate_pitch_KI", m_pitch_rate_pid_terms.y, DEFAULT_PITCH_RATE_KI);
    m_node->param("rate_pitch_KD", m_pitch_rate_pid_terms.z, DEFAULT_PITCH_RATE_KD);

    // Load the Yaw Rate PID parameters
    m_node->param("rate_yaw_KP", m_yaw_rate_pid_terms.x, DEFAULT_YAW_RATE_KP);
    m_node->param("rate_yaw_KI", m_yaw_rate_pid_terms.y, DEFAULT_YAW_RATE_KI);
    m_node->param("rate_yaw_KD", m_yaw_rate_pid_terms.z, DEFAULT_YAW_RATE_KD);

    // Load the Roll Attitude PID parameters
    m_node->param("att_roll_KP", m_roll_att_pid_terms.x, DEFAULT_ROLL_ATT_KP);
    m_node->param("att_roll_KI", m_roll_att_pid_terms.y, DEFAULT_ROLL_ATT_KI);
    m_node->param("att_roll_KD", m_roll_att_pid_terms.z, DEFAULT_ROLL_ATT_KD);

    // Load the Pitch Attitude PID parameters
    m_node->param("att_pitch_KP", m_pitch_att_pid_terms.x, DEFAULT_PITCH_ATT_KP);
    m_node->param("att_pitch_KI", m_pitch_att_pid_terms.y, DEFAULT_PITCH_ATT_KI);
    m_node->param("att_pitch_KD", m_pitch_att_pid_terms.z, DEFAULT_PITCH_ATT_KD);

    // Load the Yaw Attitude PID parameters
    m_node->param("att_yaw_KP", m_yaw_att_pid_terms.x, DEFAULT_YAW_ATT_KP);
    m_node->param("att_yaw_KI", m_yaw_att_pid_terms.y, DEFAULT_YAW_ATT_KI);
    m_node->param("att_yaw_KD", m_yaw_att_pid_terms.z, DEFAULT_YAW_ATT_KD);
}

double truncate(double value, double low, double high)
{
    if (value < low)
        value = low;
    else if (value > high)
        value = high;
    return value;
}

void FlightController::applyThrustAdjustments(Vect3F thrust_adjustments)
{
    double roll_thrust_adj, pitch_thrust_adj, yaw_thrust_adj;
    double motor1_thrust, motor2_thrust, motor3_thrust, motor4_thrust;

    simone_msgs::MotorCTRL motor_ctrl_msg;
    
    // Re-assign input vector to make it's values more comprehensive
    roll_thrust_adj  = thrust_adjustments.x;
    pitch_thrust_adj = thrust_adjustments.y;
    yaw_thrust_adj   = thrust_adjustments.z;
    
    // Calculate new thrusts

    // motor1_thrust = m_thrust + roll_thrust_adj - pitch_thrust_adj + yaw_thrust_adj;
    // motor2_thrust = m_thrust - roll_thrust_adj + pitch_thrust_adj + yaw_thrust_adj;
    // motor3_thrust = m_thrust - roll_thrust_adj - pitch_thrust_adj - yaw_thrust_adj;
    // motor4_thrust = m_thrust + roll_thrust_adj + pitch_thrust_adj - yaw_thrust_adj;

    motor4_thrust = m_thrust - roll_thrust_adj + pitch_thrust_adj + yaw_thrust_adj;
    motor2_thrust = m_thrust + roll_thrust_adj - pitch_thrust_adj + yaw_thrust_adj;
    motor3_thrust = m_thrust + roll_thrust_adj + pitch_thrust_adj - yaw_thrust_adj;
    motor1_thrust = m_thrust - roll_thrust_adj - pitch_thrust_adj - yaw_thrust_adj;

    // Make sure values do not exceed limits
    motor1_thrust = truncate(motor1_thrust, 0.0, 100.0);
    motor2_thrust = truncate(motor2_thrust, 0.0, 100.0);
    motor3_thrust = truncate(motor3_thrust, 0.0, 100.0);
    motor4_thrust = truncate(motor4_thrust, 0.0, 100.0);

    // Publish motor control message
    motor_ctrl_msg.m1 = motor1_thrust;
    motor_ctrl_msg.m2 = motor2_thrust;
    motor_ctrl_msg.m3 = motor3_thrust;
    motor_ctrl_msg.m4 = motor4_thrust;

    m_motor_publisher.publish(motor_ctrl_msg);

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
    
    // TODO: unset HARD flight mode
    m_flight_mode = STABL;

    // Run controller every time new sensor data is received.
    switch(m_flight_mode)
    {
        case ACRO:
        {
            // Calculate acro thrust adjustments
            Vect3F thrust_adjustments;
            thrust_adjustments = m_rate_controller->getOutput(rates);
            // Send thrust adjustments to motors
            this->applyThrustAdjustments(thrust_adjustments);
            break;
        }

        case STABL:
        {
            // Calculate stabalize angular velocity adjustments
            Vect3F rate_adjustments;
            rate_adjustments = m_att_controller->getOutput(orientation);
            rate_adjustments.z = m_yaw;
            // Calculate stabalize thrust adjustments
            Vect3F thrust_adjustments;
            thrust_adjustments = m_rate_controller->getOutput(rate_adjustments);
            // Send thrust adjustments to motors
            this->applyThrustAdjustments(thrust_adjustments);
            break;
        }
        
        case AUTO:
            //TODO
        
        default:
            //m_motors.killAll(); // Should be redundant
            break;
    }
}

/*void FlightController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
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
            //m_motors.killAll();
    }

}*/

void FlightController::thrustCallback(const hector_uav_msgs::ThrustCommand::ConstPtr& msg)
{
    m_thrust = 40 - (msg->thrust * 2);
}

void FlightController::yawCallback(const hector_uav_msgs::YawrateCommand::ConstPtr& msg)
{
    // Get commanded value
    m_yaw = msg->turnrate; 

    // Build control inputs from current state
    Vect3F commanded_values(m_pitch, m_roll, m_yaw); // formatted as phone's x, y, z axis

    // Update the controller with the new desired state
    m_att_controller->setDesiredAtt(commanded_values);
}

void FlightController::attitudeCallback(const hector_uav_msgs::AttitudeCommand::ConstPtr& msg)
{
    // Get commanded value
    m_roll = msg->roll * 1.5; 
    m_pitch = msg->pitch * 1.5; 

    // Build control inputs from current state
    Vect3F commanded_values(m_pitch, m_roll, m_yaw); // formatted as phone's x, y, z axis

    // Update the controller with the new desired state
    m_att_controller->setDesiredAtt(commanded_values);
}


void FlightController::dynamicReconfigureCallback(
        quadrotor_controller::flight_controllerConfig &config, 
        uint32_t level)
{
    // Re-load parameters for rate PIDs
    m_roll_rate_pid_terms.x = config.rate_roll_KP;
    m_roll_rate_pid_terms.y = config.rate_roll_KI;
    m_roll_rate_pid_terms.z = config.rate_roll_KD;
    
    m_pitch_rate_pid_terms.x = config.rate_pitch_KP;
    m_pitch_rate_pid_terms.y = config.rate_pitch_KI;
    m_pitch_rate_pid_terms.z = config.rate_pitch_KD;
    
    m_yaw_rate_pid_terms.x = config.rate_yaw_KP;
    m_yaw_rate_pid_terms.y = config.rate_yaw_KI;
    m_yaw_rate_pid_terms.z = config.rate_yaw_KD;
    
    // Re-load parameters for attitude PIDs
    m_roll_att_pid_terms.x = config.att_roll_KP;
    m_roll_att_pid_terms.y = config.att_roll_KI;
    m_roll_att_pid_terms.z = config.att_roll_KD;
    
    m_pitch_att_pid_terms.x = config.att_pitch_KP;
    m_pitch_att_pid_terms.y = config.att_pitch_KI;
    m_pitch_att_pid_terms.z = config.att_pitch_KD;
    
    m_yaw_att_pid_terms.x = config.att_yaw_KP;
    m_yaw_att_pid_terms.y = config.att_yaw_KI;
    m_yaw_att_pid_terms.z = config.att_yaw_KD;

    // Re-start PID controllers
    m_rate_controller->reset(m_roll_rate_pid_terms, m_pitch_rate_pid_terms, m_yaw_rate_pid_terms);
    m_att_controller->reset(m_roll_att_pid_terms, m_pitch_att_pid_terms, m_yaw_att_pid_terms);

    // Log the new values to console
    std::stringstream info;
    info << "\nLOADED NEW PID PARAMETERS:\n"
         << "Controller_Name\tKP\tKI\tKD\n"
         << "Roll Rate\t" 
             << m_roll_rate_pid_terms.x << "\t" 
             << m_roll_rate_pid_terms.y << "\t"
             << m_roll_rate_pid_terms.z << "\n"
         << "Pitch Rate\t"
             << m_pitch_rate_pid_terms.x << "\t"
             << m_pitch_rate_pid_terms.y << "\t"
             << m_pitch_rate_pid_terms.z << "\n"
         << "Yaw Rate\t"
             << m_yaw_rate_pid_terms.x << "\t"
             << m_yaw_rate_pid_terms.y << "\t" 
             << m_yaw_rate_pid_terms.z << "\n"
         << "Roll Att\t"
             << m_roll_att_pid_terms.x << "\t"
             << m_roll_att_pid_terms.y << "\t"
             << m_roll_att_pid_terms.z << "\n"
         << "Pitch Att\t"
             << m_pitch_att_pid_terms.x << "\t"
             << m_pitch_att_pid_terms.y << "\t"
             << m_pitch_att_pid_terms.z << "\n"
         << "Yaw Att\t\t"
             << m_yaw_att_pid_terms.x << "\t"
             << m_yaw_att_pid_terms.y << "\t"
             << m_yaw_att_pid_terms.z << "\n"
         << std::endl;
    ROS_INFO_STREAM(info.str());
}
