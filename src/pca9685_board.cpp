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

#define PIN_ALL     16
#define PIN_BASE    300
#define MAX_PWM     4096

#define I2C_ADDRESS 0x40
#define HERTZ       50


/**
 * Helper function to get to register
 */
int base_register_(const int pin)
{
    return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}

int get_int_param_(XmlRpc::XmlRpcValue obj, std::string param_name)
{
    XmlRpc::XmlRpcValue &item = obj[param_name];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return item;

    ROS_WARN("invalid paramter type for %s - expected TypeInt", param_name.c_str());
    return 0;
}


class PCA9685Board
{
public:
    PCA9685Board();
    int setup(const int i2c_address, float pwm_freq);
    void set_pwm_interval(int pin, int value);

private:
    void get_servo_params_(XmlRpc::XmlRpcValue servos);
    void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);
    void reset_all_();
    void set_pwm_freq_(float pwm_freq);
    void pwm_write_(int pin, int on, int off);
    void full_on_(int pin, int tf);
    void full_off_(int pin, int tf);

    int io_handle_;

    ros::NodeHandle nh_;
    ros::Subscriber abs_sub_;
};


PCA9685Board::PCA9685Board()
{
    if (0 > setup(I2C_ADDRESS, HERTZ))
    {
        ROS_ERROR("Error in setup.");
        return;
    }

    abs_sub_ = nh_.subscribe<pca9685_board::Servo>("servo_absolute", 1, &PCA9685Board::servo_absolute_, this);
}

void PCA9685Board::get_servo_params_(XmlRpc::XmlRpcValue servos)
{
    // if (nh_.hasParam("servos"))
    // {
    //     XmlRpc::XmlRpcValue servos_;
    //     nh_.getParam("servos", servos_);

    //     if (servos.getType() == XmlRpc::XmlRpcValue::TypeArray)
    //     {
    //         ROS_INFO(
    //             "Retrieving members from 'servos_' in namespace(%s)",
    //             nh_.getNamespace().c_str());
                
    //         for (int i = 0; i < servos.size(); ++i)
    //         {
    //             XmlRpc::XmlRpcValue servo;
    //             servo = servos[i];	// get the data from the iterator
    //             if (servo.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    //             {
    //                 ROS_INFO(
    //                     "Retrieving items from 'servo_config' member %d in namespace(%s)",
    //                     i, nh_.getNamespace().c_str());

    //                 // get the servo settings
    //                 int id, center, direction, range;
    //                 int id = get_int_param_(servo, "servo");
    //                 int center = get_int_param_(servo, "center");
    //                 int direction = get_int_param_(servo, "direction");
    //                 int range = get_int_param_(servo, "range");
                    
    //                 if (id && center && direction && range)
    //                 {
    //                     if ((id >= 1) && (id <= MAX_SERVOS))
    //                     {
    //                         int board = ((int)(id / 16)) + 1;
    //                         _set_active_board (board);
    //                         _set_pwm_frequency (pwm);
    //                         _config_servo (id, center, range, direction);
    //                     }
    //                     else
    //                     {
    //                         ROS_WARN("Parameter servo=%d is out of bounds", id);
    //                     }
    //                 }
    //                 else
    //                 {
    //                     ROS_WARN("Invalid parameters for servo=%d'", id);
    //                 }
    //             }
    //             else
    //             {
    //                 ROS_WARN(
    //                     "Invalid type %d for member of 'servo_config' - expected TypeStruct(%d)",
    //                     servo.getType(), XmlRpc::XmlRpcValue::TypeStruct);
    //             }
    //         }
    //     }
    //     else {
    //         ROS_WARN(
    //             "Invalid type %d for 'servos' - expected TypeArray(%d)",
    //             servos_.getType(), XmlRpc::XmlRpcValue::TypeArray);
    //     }
    // }
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
    PCA9685Board::set_pwm_interval(msg->servo, msg->value);
}

/**
 * Setup the PCA9685 board with wiringPi.
 *  
 * i2c_address: The default address is 0x40
 * pwm_freq:    Frequency will be capped to range [40..1000] Hertz.
 *              Try 50 for servos
 */
int PCA9685Board::setup(const int i2c_address, float pwm_freq)
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

    // Reset all output
    reset_all_();

    return fd;
}

/**
 * Simple PWM control which sets on-tick to 0 and off-tick to value.
 * If value is <= 0, full-off will be enabled
 * If value is >= 4096, full-on will be enabled
 * Every value in between enables PWM output
 */
void PCA9685Board::set_pwm_interval(int pin, int value)
{
    if (value >= 4096)
    {
        full_on_(pin, 1);
    }
    else if (value > 0)
    {
        pwm_write_(pin, 0, value);
    }
    else
    {
        full_off_(pin, 1);
    }
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca9685_board_node");
    PCA9685Board board;

    ros::spin();

    return 0;
}
