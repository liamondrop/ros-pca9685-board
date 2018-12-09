#include "pca9685_board/PCA9685Node.h"

#define I2C_ADDRESS 0x40
#define PWM_FREQ    50

/**
 * Callback function for the servos_drive topic
 * 
 * Sets the values for the steering and throttle servos using
 * a standard geometry_msgs::Twist message
 */
int get_pwm_proportional_(
    const pca9685_board::servo_config* config, const float value
)
{
    if ((value < -1.0001) || (value > 1.0001)) {
        ROS_ERROR("(%f) value invalid. Must be between -1.0 and 1.0", value);
        return 0;
    }
    int pwm = (config->direction * (((float)(config->range) / 2) * value)) + config->center;
    return pwm;
}

using namespace pca9685_board;

PCA9685Node::PCA9685Node()
{
    int pwm_freq;
    nh_.param("/servos/pwm_frequency", pwm_freq, PWM_FREQ);
    int fd = board_controller_.setup(I2C_ADDRESS, pwm_freq);
    ROS_ASSERT_MSG(0 < fd,
        "Error setting up board controller. File descriptor (%d) not valid", fd);

    configure_servo_("throttle");
    configure_servo_("steering");

    drive_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        "servos_drive", 10, &PCA9685Node::servos_drive_callback_, this);
    abs_sub_ = nh_.subscribe<pca9685_board::Servo>(
        "servo_absolute", 1, &PCA9685Node::servo_absolute_callback_, this);
}

const servo_config* PCA9685Node::get_servo_config(std::string name)
{
    if (servos_.find(name) == servos_.end()) {
        ROS_ERROR("(%s) servo is not in configured servos", name.c_str());
        return NULL;
    }
    return &servos_[name];
}

void PCA9685Node::configure_servo_(const std::string name)
{
    std::string param_name = "/servos/" + name;
    ROS_ASSERT_MSG(nh_.hasParam(param_name),
        "Param %s does not exist", param_name.c_str());
    servo_config config;
    config.center = get_int_param_(param_name + "/center");
    config.direction = get_int_param_(param_name + "/direction");
    config.pin = get_int_param_(param_name + "/pin");
    config.range = get_int_param_(param_name + "/range");
    servos_.insert(std::pair<std::string, servo_config>(name, config));

    ROS_ASSERT(servos_.find(name) != servos_.end());

    ROS_INFO("(%s) center: %d, direction: %d, pin: %d, range %d",
        name.c_str(), config.center, config.direction, config.pin,
        config.range);
}

/**
 * Function to retrieve an int value from a ROS param
 */
int PCA9685Node::get_int_param_(const std::string name)
{
    ROS_ASSERT_MSG(nh_.hasParam(name),
        "(%s) param does not exist", name.c_str());
    XmlRpc::XmlRpcValue value;
    nh_.getParam(name, value);

    ROS_ASSERT(value.getType() == XmlRpc::XmlRpcValue::TypeInt);
    return value;
}

/**
 * Function to set a given servo's pwm value based on a proportional
 * value between -1.0 and 1.0
 */
void PCA9685Node::set_servo_proportional_(
    const std::string servo_name, const float value
)
{
    const servo_config* config = get_servo_config(servo_name);
    int pwm_value = get_pwm_proportional_(config, value);
    board_controller_.set_pwm_interval(config->pin, pwm_value);
    ROS_INFO("servo: %s, pin: %d, value: %f, pwm_value: %d",
        servo_name.c_str(), config->pin, value, pwm_value);
}

/**
 * Callback function for the servos_drive topic
 * 
 * Sets the values for the steering and throttle servos using
 * a standard geometry_msgs::Twist message
 */
void PCA9685Node::servos_drive_callback_(
    const geometry_msgs::TwistConstPtr& msg
)
{
    set_servo_proportional_("steering", msg->angular.z);
    set_servo_proportional_("throttle", msg->linear.x);
}

/**
 * Callback function for the servo_absolute topic. Intended as a utility
 * to help properly configure a servo's range of pulse values
 *
 * the following messages are an example of finding the throttle servo's center, e.g. 333
 *
 * rostopic pub servo_absolute pca9685_board/Servo "{name: throttle, value: 300}"
 * rostopic pub servo_absolute pca9685_board/Servo "{name: throttle, value: 350}"
 * rostopic pub servo_absolute pca9685_board/Servo "{name: throttle, value: 330}"
 * rostopic pub servo_absolute pca9685_board/Servo "{name: throttle, value: 335}"
 * rostopic pub servo_absolute pca9685_board/Servo "{name: throttle, value: 333}"
 */
void PCA9685Node::servo_absolute_callback_(
    const pca9685_board::ServoConstPtr& msg
)
{
    const servo_config* config = get_servo_config(msg->name);
    board_controller_.set_pwm_interval(config->pin, msg->value);
    ROS_INFO("servo: %s, pin: %d, value: %d",
        msg->name.c_str(), config->pin, static_cast<int>(msg->value));
}
