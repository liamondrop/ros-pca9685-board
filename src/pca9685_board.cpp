#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// #include "i2cpwm_board/Servo.h"
// #include "i2cpwm_board/ServoArray.h"

class PCA9685Board
{
public:
    PCA9685Board();

private:
    void joyCallback(const geometry_msgs::Twist::ConstPtr& twist);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

PCA9685Board::PCA9685Board():
    linear_(1), angular_(2)
{
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_linear", a_scale_, a_scale_);
    nh_.param("scale_angular", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy", 10, &PCA9685Board::joyCallback, this);
}

void PCA9685Board::joyCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    geometry_msgs::Twist twist2;
    twist2.angular.z = a_scale_ * twist->angular.z;
    twist2.linear.x = l_scale_ * twist->linear.x;
    vel_pub_.publish(twist2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca9685_board_node");
    PCA9685Board pca9685_board_node;

    ros::spin();
}
