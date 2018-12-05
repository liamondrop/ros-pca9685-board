#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "pca9685_board/pca9685_board.h"

// Setup registers
#define PCA9685_MODE1    0x0
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L   0x6
#define LEDALL_ON_L 0xFA

#define PIN_ALL     16
#define PIN_BASE    300
#define MAX_PWM     4096


/**
 * Helper function to get to register
 */
int base_register_(const int pin)
{
    return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}

PCA9685Board::PCA9685Board(const int i2c_address)
{
    wiringPiSetupGpio();
    setup_(i2c_address);
    reset_all_();
}

PCA9685Board::~PCA9685Board()
{}

/**
 * Setup the PCA9685 board with wiringPi.
 *  
 * i2c_address: The default address is 0x40
 * pwm_freq:    Frequency will be capped to range [40..1000] Hertz.
 *              Try 50 for servos
 */
int PCA9685Board::setup_(int i2c_address)
{
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

    return fd;
}

/**
 * Set the frequency of PWM signals.
 * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
 */
void PCA9685Board::set_pwm_freq(float pwm_freq)
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
