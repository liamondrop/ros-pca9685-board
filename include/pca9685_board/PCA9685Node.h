#ifndef PCA9685Node_H_
#define PCA9685Node_H_

#include <ros/ros.h>

// #include "pca9685_board/PCA9685Controller.h"
#include "pca9685_board/Servo.h"

namespace pca9685_board
{
    typedef struct servo_config_ {
        int center;
        int direction;
        int pin;
        int range;
    } servo_config;

    class PCA9685Node
    {
    public:
        PCA9685Node();
        ~PCA9685Node() {};
        void configure_servos();

    private:
        void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);
        void configure_servo_(servo_config& servo_conf, std::string param_name);
        int get_int_param_(const std::string param_name);

        ros::NodeHandle nh_;
        ros::Subscriber abs_sub_;
        // PCA9685Controller board_controller_;
        servo_config servo_steering_;
        servo_config servo_throttle_;
    };
}

#endif  // PCA9685Node_H_
