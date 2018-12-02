#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

// Setup registers
#define PCA9685_MODE1    0x0
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L   0x6
#define LEDALL_ON_L 0xFA

#define PIN_ALL 16
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50


class PCA9685Board
{
public:
    PCA9685Board();

private:
    void joyCallback(const geometry_msgs::Twist::ConstPtr& twist);
    int setup_(const int pin_base, const int i2c_address, float pwm_freq);
    void set_pwm_freq_(const int fd, float pwm_freq);
    void reset_(const int fd);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

PCA9685Board::PCA9685Board():
    linear_(1), angular_(2)
{
    int fd = setup_(PIN_BASE, 0x40, HERTZ);
    if (fd < 0)
    {
        ROS_ERROR("Error in setup.");
        return;
    }

    // Reset all output
    reset_(fd);

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_linear", a_scale_, a_scale_);
    nh_.param("scale_angular", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy", 10, &PCA9685Board::joyCallback, this);
}

void PCA9685Board::joyCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    geometry_msgs::Twist twist2;
    twist2.angular.z = a_scale_ * twist->angular.z;
    twist2.linear.x = l_scale_ * twist->linear.x;
    vel_pub_.publish(twist2);
}


/**
 * Setup the PCA9685 board with wiringPi.
 *  
 * pin_base:    Use a pinBase > 64, eg. 300
 * i2c_address: The default address is 0x40
 * pwm_freq:    Frequency will be capped to range [40..1000] Hertz.
 *              Try 50 for servos
 */
int PCA9685Board::setup_(const int pin_base, const int i2c_address, float pwm_freq)
{
    wiringPiSetupGpio();

    // Create a node with 16 pins [0..15] + [16] for all
    struct wiringPiNodeStruct *node = wiringPiNewNode(pin_base, PIN_ALL + 1);

    // Check if pin_base is available
    if (!node) return -1;

    // Check i2c address
    int fd = wiringPiI2CSetup(i2c_address);
    if (fd < 0) return fd;

    // Setup the chip. Enable auto-increment of registers.
    int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
    int auto_inc = settings | 0x20;

    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, auto_inc);

    // Set frequency of PWM signals. Also ends sleep mode and starts PWM output.
    if (pwm_freq > 0) set_pwm_freq_(fd, pwm_freq);

    return fd;
}

/**
 * Set the frequency of PWM signals.
 * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
 */
void PCA9685Board::set_pwm_freq_(const int fd, float pwm_freq)
{
    // Cap at min and max
    pwm_freq = (pwm_freq > 1000 ? 1000 : (pwm_freq < 40 ? 40 : pwm_freq));

    // To set pwm frequency we have to set the prescale register. The formula is:
    // prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
    // Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
    int prescale = (int)(25000000.0f / (4096 * pwm_freq) - 0.5f);

    // Get settings and calc bytes for the different states.
    int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;   // Set restart bit to 0
    int sleep    = settings | 0x10;                                 // Set sleep bit to 1
    int wake     = settings & 0xEF;                                 // Set sleep bit to 0
    int restart  = wake | 0x80;                                     // Set restart bit to 1

    // Go to sleep, set prescale and wake up again.
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);
    wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);

    // Now wait a millisecond until oscillator finished stabilizing and restart PWM.
    delay(1);
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart);
}


/**
 * Set all leds back to default values (: fullOff = 1)
 */
void PCA9685Board::reset_(const int fd)
{
    wiringPiI2CWriteReg16(fd, LEDALL_ON_L, 0x0);
    wiringPiI2CWriteReg16(fd, LEDALL_ON_L + 2, 0x1000);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca9685_board_node");
    PCA9685Board pca9685_board_node;

    ros::spin();

    return 0;
}
