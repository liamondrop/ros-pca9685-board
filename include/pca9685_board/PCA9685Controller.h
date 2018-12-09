#ifndef PCA9685Controller_H_
#define PCA9685Controller_H_

#define DEFAULT_I2C_ADDR 0x40

namespace pca9685_board
{
    class PCA9685Controller
    {
    public:
        PCA9685Controller();
        ~PCA9685Controller();
        int setup(const int i2c_address = DEFAULT_I2C_ADDR);
        void set_pwm(int pin, int value);
        void set_pwm_freq(float pwm_freq);

    private:
        void full_on_(int pin, int tf);
        void full_off_(int pin, int tf);
        void pwm_write_(int pin, int on, int off);
        void reset_all_();

        int io_handle_;
    };
}

#endif  // PCA9685Controller_H_
