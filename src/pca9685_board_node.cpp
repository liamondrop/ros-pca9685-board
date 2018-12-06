#include <ros/ros.h>

#include "pca9685_board/pca9685_board.h"
#include "pca9685_board/Servo.h"

#define I2C_ADDRESS 0x40
#define PWM_FREQ    50


typedef struct _servo_config {
    int center;
    int direction;
    int pin;
    int range;
} servo_config;


int get_int_param_(XmlRpc::XmlRpcValue obj, std::string param_name)
{
    XmlRpc::XmlRpcValue &value = obj[param_name];
    ROS_ASSERT(value.getType() == XmlRpc::XmlRpcValue::TypeInt);
    return value;
}


class PCA9685BoardNode
{
public:
    PCA9685BoardNode();
    ~PCA9685BoardNode() {};
    void configure_servos();

private:
    void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);
    void configure_servo_(servo_config& servo_conf, std::string param_name);

    ros::NodeHandle nh_;
    ros::Subscriber abs_sub_;
    PCA9685Board board_;
    servo_config servo_steering_;
    servo_config servo_throttle_;
};


PCA9685BoardNode::PCA9685BoardNode()
{
    board_.setup(I2C_ADDRESS, PWM_FREQ);
    configure_servo_(servo_throttle_, "/servos/throttle");
    configure_servo_(servo_steering_, "/servos/steering");

    abs_sub_ = nh_.subscribe<pca9685_board::Servo>(
        "servo_absolute", 1, &PCA9685BoardNode::servo_absolute_, this);
}

void PCA9685BoardNode::configure_servo_(servo_config& config, std::string servo_name)
{
    ROS_ASSERT(nh_.hasParam(servo_name));
    XmlRpc::XmlRpcValue servo_params;
    nh_.getParam(servo_name, servo_params);

    config.center = get_int_param_(servo_params, "center");
    config.direction = get_int_param_(servo_params, "direction");
    config.pin = get_int_param_(servo_params, "pin");
    config.range = get_int_param_(servo_params, "range");

    ROS_INFO("SERVO (%s) - CENTER: %d, DIRECTION: %d, PIN: %d, RANGE %d",
        servo_name.c_str(), config.center, config.direction,
        config.pin, config.range);
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
 */
void PCA9685BoardNode::servo_absolute_(const pca9685_board::Servo::ConstPtr& msg)
{
    board_.set_pwm_interval(msg->servo, msg->value);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca9685_board_node");
    PCA9685BoardNode node;
    ros::spin();
    return 0;
}
