#ifndef PCA9685Node_H_
#define PCA9685Node_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "pca9685_board/PCA9685Controller.h"
#include "pca9685_board/Servo.h"

namespace pca9685_board
{
    typedef struct servo_config_ {
        int center;
        int channel;
        int direction;
        int range;
    } servo_config;

    class PCA9685Node
    {
    public:
        PCA9685Node();
        ~PCA9685Node() {};
        const servo_config* get_servo_config(std::string name);

    private:
        void configure_servo_(const std::string name);
        int get_int_param_(const std::string name);
        void servo_absolute_callback_(const pca9685_board::ServoConstPtr& msg);
        void servos_drive_callback_(const geometry_msgs::TwistConstPtr& msg);
        void set_servo_proportional_(const std::string servo_name, const float value);

        PCA9685Controller board_controller_;
        ros::NodeHandle nh_;
        ros::Subscriber abs_sub_;
        ros::Subscriber drive_sub_;
        std::map<std::string, servo_config> servos_;
    };
}

#endif  // PCA9685Node_H_
