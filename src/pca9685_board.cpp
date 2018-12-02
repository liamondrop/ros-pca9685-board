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

#define PIN_ALL  16
#define PIN_BASE 300
#define MAX_PWM  4096
#define HERTZ    50


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
    void pwm_write(int pin, int value);

private:
    void joyCallback(const geometry_msgs::Twist::ConstPtr& twist);
    int  setup_(const int i2c_address, float pwm_freq);
    void set_pwm_freq_(float pwm_freq);
    void reset_();
    void pwm_write_(int pin, int on, int off);
    void full_on_(int pin, int tf);
    void full_off_(int pin, int tf);

    ros::NodeHandle nh_;

    int fd_;
    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};


PCA9685Board::PCA9685Board():
    linear_(1), angular_(2)
{
    int fd = setup_(0x40, HERTZ);
    if (fd < 0)
    {
        ROS_ERROR("Error in setup.");
        return;
    }

    // Reset all output
    reset_();

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
 * Simple PWM control which sets on-tick to 0 and off-tick to value.
 * If value is <= 0, full-off will be enabled
 * If value is >= 4096, full-on will be enabled
 * Every value in between enables PWM output
 */
void PCA9685Board::pwm_write(int pin, int value)
{
    int ipin = pin - PIN_BASE;
    if (value >= 4096)
        full_on_(ipin, 1);
    else if (value > 0)
        pwm_write_(ipin, 0, value);	// (Deactivates full-on and off by itself)
    else
        full_off_(ipin, 1);
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
    fd_ = fd;

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
    int settings = wiringPiI2CReadReg8(fd_, PCA9685_MODE1) & 0x7F;   // Set restart bit to 0
    int sleep    = settings | 0x10;                                 // Set sleep bit to 1
    int wake     = settings & 0xEF;                                 // Set sleep bit to 0
    int restart  = wake | 0x80;                                     // Set restart bit to 1

    // Go to sleep, set prescale and wake up again.
    wiringPiI2CWriteReg8(fd_, PCA9685_MODE1, sleep);
    wiringPiI2CWriteReg8(fd_, PCA9685_PRESCALE, prescale);
    wiringPiI2CWriteReg8(fd_, PCA9685_MODE1, wake);

    // Now wait a millisecond until oscillator finished
    // stabilizing and restart PWM.
    delay(1);
    wiringPiI2CWriteReg8(fd_, PCA9685_MODE1, restart);
}

/**
 * Set all leds back to default values (: fullOff = 1)
 */
void PCA9685Board::reset_()
{
    wiringPiI2CWriteReg16(fd_, LEDALL_ON_L, 0x0);
    wiringPiI2CWriteReg16(fd_, LEDALL_ON_L + 2, 0x1000);
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
    wiringPiI2CWriteReg16(fd_, reg, on  & 0x0FFF);
    wiringPiI2CWriteReg16(fd_, reg + 2, off & 0x0FFF);
}

/**
 * Enables or deactivates full-on
 * tf = true: full-on
 * tf = false: according to PWM
 */
void PCA9685Board::full_on_(int pin, int tf)
{
    int reg = base_register_(pin) + 1;  // LEDX_ON_H
    int state = wiringPiI2CReadReg8(fd_, reg);

    // Set bit 4 to 1 or 0 accordingly
    state = tf ? (state | 0x10) : (state & 0xEF);

    wiringPiI2CWriteReg8(fd_, reg, state);

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
    int state = wiringPiI2CReadReg8(fd_, reg);

    // Set bit 4 to 1 or 0 accordingly
    state = tf ? (state | 0x10) : (state & 0xEF);

    wiringPiI2CWriteReg8(fd_, reg, state);
}

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

    // Set servo to neutral position at 1.5 milliseconds
    // (View http://en.wikipedia.org/wiki/Servo_control#Pulse_duration)
    float millis = 1.5;
    int tick = calcTicks(millis, HERTZ);
    board.pwm_write(PIN_BASE + 16, tick);

    ros::Rate r(1); // 1 hz
    while (ros::ok())
    {
        // ... do some work, publish some messages, etc. ...
        // That's a hack. We need a random number < 1
        float r = rand();
        while (r > 1) r /= 10;

        millis = map(r, 1, 2);
        tick = calcTicks(millis, HERTZ);

        ROS_INFO("Tick: %d", tick);

        board.pwm_write(PIN_BASE + 16, tick);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
