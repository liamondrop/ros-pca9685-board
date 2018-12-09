#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "pca9685_board/PCA9685Controller.h"


#define PIN_ALL      16
#define PIN_BASE     300
#define MAX_PWM      4096
#define MIN_PWM_FREQ 40
#define MAX_PWM_FREQ 1000

// Setup registers, etc.
enum pca9685_reg {
    MODE1              = 0x00,
    MODE2              = 0x01,
    CHANNEL_ON_L       = 0x06,
    CHANNEL_ON_H       = 0x07,
    ALL_CHANNELS_ON_L  = 0xFA,
    ALL_CHANNELS_ON_H  = 0xFB,
    PRESCALE           = 0xFE,
    RESTART            = 0x80,
    SLEEP              = 0x10,
    WAKE               = 0xEF
};

using namespace pca9685_board;

/**
 * Helper function to get to register
 */
int base_register_(const int pin)
{
    return (pin >= PIN_ALL ? ALL_CHANNELS_ON_L : CHANNEL_ON_L + 4 * pin);
}

PCA9685Controller::PCA9685Controller()
{
    // initialize wiringPi lib
    wiringPiSetupGpio();
}

PCA9685Controller::~PCA9685Controller()
{
    // Set all channels back to their defaults
    reset_all_();
}

/**
 * Setup the PCA9685 board with wiringPi.
 *  
 * i2c_address: The address of the board on the i2c bus. Defaults to 0x40.
 */
int PCA9685Controller::setup(const int i2c_address)
{
    // Create a node with 16 pins [0..15] + [16] for all
    struct wiringPiNodeStruct *node = wiringPiNewNode(PIN_BASE, PIN_ALL + 1);

    // Check if pin_base is available
    if (!node) return -1;

    // Check i2c address
    io_handle_ = wiringPiI2CSetup(i2c_address);
    if (io_handle_ < 0) return io_handle_;

    // Setup the chip. Enable auto-increment of registers.
    int settings = wiringPiI2CReadReg8(io_handle_, MODE1) & 0x7F;
    int auto_inc = settings | 0x20;
    wiringPiI2CWriteReg8(io_handle_, MODE1, auto_inc);

    reset_all_();
    return io_handle_;
}

/**
 * Simple PWM control which sets on-tick to 0 and off-tick to value.
 * If value is <= 0, full-off will be enabled
 * If value is >= 4096, full-on will be enabled
 * Every value in between enables PWM output
 */
void PCA9685Controller::set_pwm(int pin, int value)
{
    if (value >= MAX_PWM) full_on_(pin, 1);
    else if (value > 0)   pwm_write_(pin, 0, value);
    else                  full_off_(pin, 1);
}

/**
 * Set the frequency of PWM signals.
 * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
 */
void PCA9685Controller::set_pwm_freq(float pwm_freq)
{
    // Cap at min and max
    pwm_freq = (pwm_freq > MAX_PWM_FREQ ? MAX_PWM_FREQ :
               (pwm_freq < MIN_PWM_FREQ ? MIN_PWM_FREQ : pwm_freq));

    // To set pwm frequency we have to set the prescale register. The formula is:
    // prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
    // Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
    int prescale = (int)(25000000.0f / (MAX_PWM * pwm_freq) - 0.5f);

    // Get settings and calc bytes for the different states.
    int settings = wiringPiI2CReadReg8(io_handle_, MODE1) & 0x7F; // Set restart bit to 0
    int sleep    = settings | SLEEP; // Set sleep bit to 1
    int wake     = settings & WAKE;  // Set sleep bit to 0
    int restart  = wake | RESTART;   // Set restart bit to 1

    // Go to sleep, set prescale and wake up again.
    wiringPiI2CWriteReg8(io_handle_, MODE1, sleep);
    wiringPiI2CWriteReg8(io_handle_, PRESCALE, prescale);
    wiringPiI2CWriteReg8(io_handle_, MODE1, wake);

    // Now wait a millisecond until oscillator finished
    // stabilizing and restart PWM.
    delay(1);
    wiringPiI2CWriteReg8(io_handle_, MODE1, restart);
}

/**
 * Enables or deactivates full-off
 * tf = true: full-off
 * tf = false: according to PWM or full-on
 */
void PCA9685Controller::full_off_(int pin, int tf)
{
    int reg   = base_register_(pin) + 3;  // CHANNELX_OFF_H
    int state = wiringPiI2CReadReg8(io_handle_, reg);

    // Set bit 4 to 1 or 0 accordingly
    state = tf ? (state | SLEEP) : (state & WAKE);

    wiringPiI2CWriteReg8(io_handle_, reg, state);
}

/**
 * Enables or deactivates full-on
 * tf = true: full-on
 * tf = false: according to PWM
 */
void PCA9685Controller::full_on_(int pin, int tf)
{
    int reg = base_register_(pin) + 1;  // CHANNELX_ON_H
    int state = wiringPiI2CReadReg8(io_handle_, reg);

    // Set bit 4 to 1 or 0 accordingly
    state = tf ? (state | SLEEP) : (state & WAKE);

    wiringPiI2CWriteReg8(io_handle_, reg, state);

    // For simplicity, we set full-off to 0
    // because it has priority over full-on
    if (tf) full_off_(pin, 0);
}

/**
 * Write on and off ticks manually to a pin
 * (Deactivates any full-on and full-off)
 */
void PCA9685Controller::pwm_write_(int pin, int on, int off)
{
    int reg = base_register_(pin);

    // Write to on and off registers and mask the
    // 12 lowest bits of data to overwrite full-on and off
    wiringPiI2CWriteReg16(io_handle_, reg,     on & 0x0FFF);
    wiringPiI2CWriteReg16(io_handle_, reg + 2, off & 0x0FFF);
}

/**
 * Set all channels back to default values (: fullOff = 1)
 */
void PCA9685Controller::reset_all_()
{
    wiringPiI2CWriteReg16(io_handle_, ALL_CHANNELS_ON_L,     0x0);
    wiringPiI2CWriteReg16(io_handle_, ALL_CHANNELS_ON_L + 2, 0x1000);
}
