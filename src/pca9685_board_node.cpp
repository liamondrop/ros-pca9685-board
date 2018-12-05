#include <ros/ros.h>

#include "pca9685_board/pca9685_board.h"

#include "pca9685_board/Servo.h"

#define I2C_ADDRESS 0x40
#define PWM_FREQ    50


int get_int_param_(XmlRpc::XmlRpcValue obj, std::string param_name)
{
    XmlRpc::XmlRpcValue &value = obj[param_name];
    ROS_ASSERT(item.getType() == XmlRpc::XmlRpcValue::TypeInt);
    return static_cast<int>(value);
}


// class PCA9685BoardNode
// {
// public:
//     PCA9685BoardNode();

// private:
//     void get_servo_params_(XmlRpc::XmlRpcValue servos);
//     void servo_absolute_(const pca9685_board::Servo::ConstPtr& msg);

//     ros::NodeHandle nh_;
//     ros::Subscriber abs_sub_;
// };


// PCA9685Board::PCA9685Board()
// {
//     if (0 > setup(I2C_ADDRESS, PWM_FREQ))
//     {
//         ROS_ERROR("Error in setup.");
//         return;
//     }

//     abs_sub_ = nh_.subscribe<pca9685_board::Servo>("servo_absolute", 1, &PCA9685Board::servo_absolute_, this);
// }

void get_servo_params_(XmlRpc::XmlRpcValue servos)
{
    // if (nh_.hasParam("servos"))
    // {
    //     XmlRpc::XmlRpcValue servos_;
    //     nh_.getParam("servos", servos_);

    //     if (servos.getType() == XmlRpc::XmlRpcValue::TypeArray)
    //     {
    //         ROS_INFO(
    //             "Retrieving members from 'servos_' in namespace(%s)",
    //             nh_.getNamespace().c_str());
                
    //         for (int i = 0; i < servos.size(); ++i)
    //         {
    //             XmlRpc::XmlRpcValue servo;
    //             servo = servos[i];	// get the data from the iterator
    //             if (servo.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    //             {
    //                 ROS_INFO(
    //                     "Retrieving items from 'servo_config' member %d in namespace(%s)",
    //                     i, nh_.getNamespace().c_str());

    //                 // get the servo settings
    //                 int id, center, direction, range;
    //                 int id = get_int_param_(servo, "servo");
    //                 int center = get_int_param_(servo, "center");
    //                 int direction = get_int_param_(servo, "direction");
    //                 int range = get_int_param_(servo, "range");
                    
    //                 if (id && center && direction && range)
    //                 {
    //                     if ((id >= 1) && (id <= MAX_SERVOS))
    //                     {
    //                         int board = ((int)(id / 16)) + 1;
    //                         _set_active_board (board);
    //                         _set_pwm_frequency (pwm);
    //                         _config_servo (id, center, range, direction);
    //                     }
    //                     else
    //                     {
    //                         ROS_WARN("Parameter servo=%d is out of bounds", id);
    //                     }
    //                 }
    //                 else
    //                 {
    //                     ROS_WARN("Invalid parameters for servo=%d'", id);
    //                 }
    //             }
    //             else
    //             {
    //                 ROS_WARN(
    //                     "Invalid type %d for member of 'servo_config' - expected TypeStruct(%d)",
    //                     servo.getType(), XmlRpc::XmlRpcValue::TypeStruct);
    //             }
    //         }
    //     }
    //     else {
    //         ROS_WARN(
    //             "Invalid type %d for 'servos' - expected TypeArray(%d)",
    //             servos_.getType(), XmlRpc::XmlRpcValue::TypeArray);
    //     }
    // }
}

// /**
//  * Subscriber for the servo_absolute topic which processes a servo and sets its physical pulse value.
//  *
//  * the following messages are an example of finding a continuous servo's center
//  * in this example the center is found to be 333
//  * 
//  * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 300}"
//  * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 350}"
//  * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 320}"
//  * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 330}"
//  * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 335}"
//  * rostopic pub servo_absolute pca9685_board/Servo "{servo: 1, value: 333}"
//  * rostopic pub -1 /servos_absolute i2cpwm_board/ServoArray "{servos:[{servo: 1, value: 300}]}"
//  */
// void PCA9685Board::servo_absolute_(const pca9685_board::Servo::ConstPtr& msg)
// {
//     PCA9685Board::set_pwm_interval(msg->servo, msg->value);
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca9685_board_node");
    PCA9685Board board;
    board.setup(I2C_ADDRESS);
    board.set_pwm_freq(PWM_FREQ);

    ros::spin();

    return 0;
}
