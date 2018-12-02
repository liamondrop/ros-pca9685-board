#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "pca9685_board/Servo.h"

// Setup registers
#define PCA9685_MODE1    0x0
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L   0x6
#define LEDALL_ON_L 0xFA

#define I2C_ADDRESS 0x40
#define PIN_ALL     16
#define PIN_BASE    300
#define MAX_PWM     4096
#define HERTZ       50


/**
 * Helper function to get to register
 */
int base_register_(const int pin)
{
	return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}


class PCA9685Board
{
public:
    PCA9685Board();
    int pulse;
    int servo;

private:
    void joyCallback(const geometry_msgs::Twist::ConstPtr& twist);
    void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);
    int  setup_(const int i2c_address, float pwm_freq);
    void reset_all_();
    void set_pwm_freq_(float pwm_freq);
    void set_pwm_interval_(int pin, int value);
    void pwm_write_(int pin, int on, int off);
    void full_on_(int pin, int tf);
    void full_off_(int pin, int tf);

    ros::NodeHandle nh_;

    int io_handle_;
    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber abs_sub_;
};


PCA9685Board::PCA9685Board()
{
    if (0 > setup_(I2C_ADDRESS, HERTZ))
    {
        ROS_ERROR("Error in setup.");
        return;
    }

    // Reset all output
    reset_all_();

    nh_.param("scale_linear", a_scale_, a_scale_);
    nh_.param("scale_angular", l_scale_, l_scale_);
    nh_.param("/pca9685_board_node/servo", servo, servo);
    nh_.param("/pca9685_board_node/pulse", pulse, pulse);
    joy_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy", 10, &PCA9685Board::joyCallback, this);
    abs_sub_ = nh_.subscribe<pca9685_board::Servo>("servo_absolute", 1, &PCA9685Board::servo_absolute_, this);
}

void PCA9685Board::joyCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    geometry_msgs::Twist twist2;
    twist2.angular.z = a_scale_ * twist->angular.z;
    twist2.linear.x = l_scale_ * twist->linear.x;
}

/**
 * Subscriber for the servo_absolute topic which processes a servo and sets its physical pulse value.
 *
 * the following messages are an example of finding a continuous servo's center
 * in this example the center is found to be 333
 * 
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 300}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 350}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 320}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 330}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 335}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 333}"
 * rostopic pub -1 /servos_absolute i2cpwm_board/ServoArray "{servos:[{servo: 1, value: 300}]}"
 */
void PCA9685Board::servo_absolute_(const pca9685_board::Servo::ConstPtr& msg)
{
    PCA9685Board::set_pwm_interval_(msg->servo, msg->value);
}

/**
 * Setup the PCA9685 board with wiringPi.
 *  
 * pin_base:    Use a pinBase > 64, eg. 300
 * i2c_address: The default address is 0x40
 * pwm_freq:    Frequency will be capped to range [40..1000] Hertz.
 *              Try 50 for servos
 */
int PCA9685Board::setup_(const int i2c_address, float pwm_freq)
{
    wiringPiSetupGpio();

    // Create a node with 16 pins [0..15] + [16] for all
    struct wiringPiNodeStruct *node = wiringPiNewNode(PIN_BASE, PIN_ALL + 1);

    // Check if pin_base is available
    if (!node) return -1;

    // Check i2c address
    int fd = wiringPiI2CSetup(i2c_address);
    if (fd < 0) return fd;
    io_handle_ = fd;

    // Setup the chip. Enable auto-increment of registers.
    int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
    int auto_inc = settings | 0x20;

    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, auto_inc);

    // Set frequency of PWM signals. Also ends sleep mode and starts PWM output.
    if (pwm_freq > 0) set_pwm_freq_(pwm_freq);

    return fd;
}

/**
 * Set the frequency of PWM signals.
 * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
 */
void PCA9685Board::set_pwm_freq_(float pwm_freq)
{
    // Cap at min and max
    pwm_freq = (pwm_freq > 1000 ? 1000 : (pwm_freq < 40 ? 40 : pwm_freq));

    // To set pwm frequency we have to set the prescale register. The formula is:
    // prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
    // Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
    int prescale = (int)(25000000.0f / (4096 * pwm_freq) - 0.5f);

    // Get settings and calc bytes for the different states.
    int settings = wiringPiI2CReadReg8(io_handle_, PCA9685_MODE1) & 0x7F;   // Set restart bit to 0
    int sleep    = settings | 0x10;                                 // Set sleep bit to 1
    int wake     = settings & 0xEF;                                 // Set sleep bit to 0
    int restart  = wake | 0x80;                                     // Set restart bit to 1

    // Go to sleep, set prescale and wake up again.
    wiringPiI2CWriteReg8(io_handle_, PCA9685_MODE1, sleep);
    wiringPiI2CWriteReg8(io_handle_, PCA9685_PRESCALE, prescale);
    wiringPiI2CWriteReg8(io_handle_, PCA9685_MODE1, wake);

    // Now wait a millisecond until oscillator finished
    // stabilizing and restart PWM.
    delay(1);
    wiringPiI2CWriteReg8(io_handle_, PCA9685_MODE1, restart);
}

/**
 * Simple PWM control which sets on-tick to 0 and off-tick to value.
 * If value is <= 0, full-off will be enabled
 * If value is >= 4096, full-on will be enabled
 * Every value in between enables PWM output
 */
void PCA9685Board::set_pwm_interval_(int pin, int value)
{
    if (value >= 4096)
        full_on_(pin, 1);
        ROS_INFO("SERVO[%d] = full_on", pin);
    else if (value > 0)
        pwm_write_(pin, 0, value);	// (Deactivates full-on and off by itself)
        ROS_INFO("SERVO[%d] = %d", pin, value);
    else
        full_off_(pin, 1);
        ROS_INFO("SERVO[%d] = full_off", pin);
}

/**
 * Set all leds back to default values (: fullOff = 1)
 */
void PCA9685Board::reset_all_()
{
    wiringPiI2CWriteReg16(io_handle_, LEDALL_ON_L,     0x0);
    wiringPiI2CWriteReg16(io_handle_, LEDALL_ON_L + 2, 0x1000);
}

/**
 * Write on and off ticks manually to a pin
 * (Deactivates any full-on and full-off)
 */
void PCA9685Board::pwm_write_(int pin, int on, int off)
{
    int reg = base_register_(pin);

    // Write to on and off registers and mask the
    // 12 lowest bits of data to overwrite full-on and off
    wiringPiI2CWriteReg16(io_handle_, reg,     on & 0x0FFF);
    wiringPiI2CWriteReg16(io_handle_, reg + 2, off & 0x0FFF);
}

/**
 * Enables or deactivates full-on
 * tf = true: full-on
 * tf = false: according to PWM
 */
void PCA9685Board::full_on_(int pin, int tf)
{
    int reg = base_register_(pin) + 1;  // LEDX_ON_H
    int state = wiringPiI2CReadReg8(io_handle_, reg);

    // Set bit 4 to 1 or 0 accordingly
    state = tf ? (state | 0x10) : (state & 0xEF);

    wiringPiI2CWriteReg8(io_handle_, reg, state);

    // For simplicity, we set full-off to 0
    // because it has priority over full-on
    if (tf) full_off_(pin, 0);
}

/**
 * Enables or deactivates full-off
 * tf = true: full-off
 * tf = false: according to PWM or full-on
 */
void PCA9685Board::full_off_(int pin, int tf)
{
    int reg = base_register_(pin) + 3;  // LEDX_OFF_H
    int state = wiringPiI2CReadReg8(io_handle_, reg);

    // Set bit 4 to 1 or 0 accordingly
    state = tf ? (state | 0x10) : (state & 0xEF);

    wiringPiI2CWriteReg8(io_handle_, reg, state);
}

// /**
//  * \private method to set a value for a PWM channel, based on a range of ±1.0, on the active board
//  *
//  *The pulse defined by start/stop will be active on the specified servo channel until any subsequent call changes it.
//  *@param servo an int value (1..16) indicating which channel to change power
//  *@param value an int value (±1.0) indicating when the size of the pulse for the channel.
//  *Example _set_pwm_interval (3, 0, 350)    // set servo #3 (fourth position on the hardware board) with a pulse of 350
//  */
// void _set_pwm_interval_proportional_(int servo, float value)
// {
//     // need a little wiggle room to allow for accuracy of a
//     // floating point value
//     if ((value < -1.0001) || (value > 1.0001)) {
//         ROS_ERROR("Invalid proportion value %f :: proportion \
//             values must be between -1.0 and 1.0", value);
//         return;
//     }

//     servo_config* configp = &(_servo_configs[servo-1]);

//     if ((configp->center < 0) ||(configp->range < 0)) {
//         ROS_ERROR("Missing servo configuration for servo[%d]", servo);
//         return;
//     }

//     int pos = (configp->direction * (((float)(configp->range) / 2) * value)) +
//         configp->center;

//     if ((pos < 0) || (pos > 4096)) {
//         ROS_ERROR(
//             "Invalid computed position servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d",
//             servo, configp->direction, configp->range, value, configp->center, pos);
//         return;
//     }
//     _set_pwm_interval(servo, 0, pos);
//     ROS_DEBUG(
//         "servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d",
//         servo, configp->direction, configp->range, value, configp->center, pos);
// }

// void servos_proportional(const i2cpwm_board::ServoArray::ConstPtr& msg)
// {
//     /* this subscription works on the active_board */

//     for (std::vector<i2cpwm_board::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
//         int servo = sp->servo;
//         float value = sp->value;
//         ROS_INFO("servo[%d] = %d", servo, value);
//         _set_pwm_interval_proportional(servo, value);
//     }
// }

/**
 * Calculate the number of ticks the signal should be high
 * for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

/**
 * input is [0..1]
 * output is [min..max]
 */
float map(float input, float min, float max)
{
	return (input * max) + (1 - input) * min;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca9685_board_node");
    PCA9685Board board;

    // // Set servo to neutral position at 1.5 milliseconds
    // // (View http://en.wikipedia.org/wiki/Servo_control#Pulse_duration)
    // float millis = 1.5;
    // int tick = calcTicks(millis, HERTZ);
    // board.pwm_write(PIN_ALL, tick);

    // ROS_INFO("Tick: %d", tick);
    // ROS_INFO("Pulse: %d", board.pulse);

    // board.pwm_write(board.servo, board.pulse);

    ros::spin();

    return 0;
}
