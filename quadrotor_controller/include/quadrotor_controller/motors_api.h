/**
 * @file motors_api.h
 * @brief Provides a place-holder motor api for a quadrotor
 * @author Jeremy Roy <jeremy.roy@queensu.ca>
 * @Copyright (c) Jeremy Roy, 2018
 */


#include <sstream>
#include <thread>
#include <chrono>

#include <ros/console.h>

#define MOTOR_1     0
#define MOTOR_2     1
#define MOTOR_3     2
#define MOTOR_4     3

// TODO: Figure out what these values are IRL
#define MIN_DUTY    2.0
#define MAX_DUTY    8.0
#define DELTA_DUTY  (MAX_DUTY - MIN_DUTY)


/**
 * @class Motors
 * @brief Dummy motors class that prints the motor's duty to console
 */
class Motors
{
public:
    Motors(void); // Constructor
    ~Motors(void); // Destructor

    bool setMotorThrust(int motor, int thrust);
    void killAll(void);

private:
    double m1_duty, m2_duty, m3_duty, m4_duty;
    bool m_kill_thread;
    std::thread m_print_thread;

    void printDuty(void);
};


/**
 * @brief Constructor.  Start the print thread
 */
Motors::Motors(void) : m1_duty(0), m2_duty(0), m3_duty(0), m4_duty(0)
{
    // Start print thread
    m_kill_thread = false;
    m_print_thread = std::thread(&Motors::printDuty, this);
}


/**
 * @brief Destructor.  Kill the print thread
 */
Motors::~Motors(void)
{
    m_kill_thread = true;
    m_print_thread.join();
}


/**
 * @brief Sets motor duty cycle
 * @param motor   Specifies which motor to set new duty to
 * @param thrust  Full scale motor thrust from 0 to 100 %
 */
bool Motors::setMotorThrust(int motor, int thrust)
{
    double duty (0);

    // Clamp thrust
    if ( thrust < 0)
    {
        thrust = 0;
    }
    else if (thrust > 100)
    {
        thrust = 100;
    }

    // Scale thrust to duty
    duty = (thrust * DELTA_DUTY)/100.0 + MIN_DUTY;
    //duty = thrust;

    // Set duty to motor
    switch(motor)
    {
        case MOTOR_1:
            // Set motor 1 duty to provided value.
            m1_duty = duty;
        case MOTOR_2:
            // Set motor 2 duty to provided value.
            m2_duty = duty;
        case MOTOR_3:
            // Set motor 3 duty to provided value.
            m3_duty = duty;
        case MOTOR_4:
            // Set motor 4 duty to provided value.
            m4_duty = duty;
        default:
            // Invalid motor
            return false;
    }
    return true; // Return true if duty successfully set
}


void Motors::killAll(void)
{
    m1_duty = 0.0;
    m2_duty = 0.0;
    m3_duty = 0.0;
    m4_duty = 0.0;
}


/**
 * @brief Print the duty as long as print thread is alive
 */
void Motors::printDuty(void)
{
    while (!m_kill_thread)
    {
        std::stringstream output;
        output << "Motor 1 Duty: " << m1_duty << "%\t"
               << "Motor 2 Duty: " << m2_duty << "%\t"
               << "Motor 3 Duty: " << m3_duty << "%\t"
               << "Motor 4 Duty: " << m4_duty << "%";
        ROS_INFO_STREAM(output.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
