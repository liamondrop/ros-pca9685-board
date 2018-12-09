#ifndef PCA9685Node_H_
#define PCA9685Node_H_

#include <ros/ros.h>

#include "pca9685_board/PCA9685Controller.h"
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
        const servo_config* get_servo_config(std::string name);

    private:
        void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);
        void servo_proportional_(const pca9685_board::Servo::ConstPtr& msg);
        void configure_servo_(std::string name);
        int get_int_param_(std::string name);

        ros::NodeHandle nh_;
        ros::Subscriber abs_sub_;
        ros::Subscriber prop_sub_;
        PCA9685Controller board_controller_;
        std::map<std::string, servo_config> servos_;
    };
}

#endif  // PCA9685Node_H_
