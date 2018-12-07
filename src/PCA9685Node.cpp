#include "pca9685_board/PCA9685Node.h"

#define I2C_ADDRESS 0x40
#define PWM_FREQ    50

using namespace pca9685_board;

PCA9685Node::PCA9685Node()
{
    ROS_ASSERT(0 < board_controller_.setup(I2C_ADDRESS, PWM_FREQ));

    configure_servo_(servo_throttle_, "/servos/throttle");
    configure_servo_(servo_steering_, "/servos/steering");

    abs_sub_ = nh_.subscribe<pca9685_board::Servo>(
        "servo_absolute", 1, &PCA9685Node::servo_absolute_, this);
}

void PCA9685Node::configure_servo_(
    pca9685_board::servo_config& config, std::string servo_name
)
{
    config.center = get_int_param_(servo_name + "/center");
    config.direction = get_int_param_(servo_name + "/direction");
    config.pin = get_int_param_(servo_name + "/pin");
    config.range = get_int_param_(servo_name + "/range");

    ROS_INFO("(%s) CENTER: %d, DIRECTION: %d, PIN: %d, RANGE %d",
        servo_name.c_str(), config.center, config.direction,
        config.pin, config.range);
}

int PCA9685Node::get_int_param_(const std::string param_name)
{
    ROS_ASSERT(nh_.hasParam(param_name));
    XmlRpc::XmlRpcValue value;
    nh_.getParam(param_name, value);

    ROS_ASSERT(value.getType() == XmlRpc::XmlRpcValue::TypeInt);
    return value;
}

/**
 * Utility function subscribing to the servo_absolute topic
 * in order to set a servo's pulse value.
 *
 * the following messages are an example of finding the servo's center,
 * which this example is 333
 *
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 300}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 350}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 330}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 335}"
 * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 333}"
 */
void PCA9685Node::servo_absolute_(const pca9685_board::Servo::ConstPtr& msg)
{
    board_controller_.set_pwm_interval(msg->servo, msg->value);
}
