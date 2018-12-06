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

servo_config get_servo_config_(XmlRpc::XmlRpcValue obj, std::string param_name)
{
    ROS_ASSERT(obj.hasMember(param_name));
    XmlRpc::XmlRpcValue& value = obj[param_name];
    ROS_ASSERT(value.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    servo_config config;
    config.center = get_int_param_(value, "center");
    config.direction = get_int_param_(value, "direction");
    config.pin = get_int_param_(value, "pin");
    config.range = get_int_param_(value, "range");
    return config;
}


class PCA9685BoardNode
{
public:
    PCA9685BoardNode();
    ~PCA9685BoardNode() {};
    void configure_servos();

private:
    void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber abs_sub_;
    PCA9685Board board_;
    servo_config servo_steering_;
    servo_config servo_throttle_;
};


PCA9685BoardNode::PCA9685BoardNode()
{
    board_.setup(I2C_ADDRESS, PWM_FREQ);
    abs_sub_ = nh_.subscribe<pca9685_board::Servo>("servos_absolute", 1);
}

void PCA9685BoardNode::configure_servos()
{
    ROS_ASSERT(nh_.hasParam("servos"));

    XmlRpc::XmlRpcValue servos;
    nh_.getParam("servos", servos);

    ROS_ASSERT(servos.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    servo_throttle_ = get_servo_config_(servos, "throttle");
    servo_steering_ = get_servo_config_(servos, "steering");
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
 * rostopic pub -1 /servos_absolute i2cpwm_board/ServoArray "{servos:[{servo: 1, value: 300}]}"
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
