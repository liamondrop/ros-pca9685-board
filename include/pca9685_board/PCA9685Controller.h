#ifndef PCA9685Controller_H_
#define PCA9685Controller_H_

namespace pca9685_board
{
    class PCA9685Controller
    {
    public:
        PCA9685Controller();
        ~PCA9685Controller();
        int setup(const int i2c_address, float pwm_freq);
        void set_pwm(int pin, int value);

    private:
        void reset_all_();
        void set_pwm_freq_(float pwm_freq);
        void pwm_write_(int pin, int on, int off);
        void full_on_(int pin, int tf);
        void full_off_(int pin, int tf);

        int io_handle_;
    };
}

#endif  // PCA9685Controller_H_
