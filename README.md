# ROS PCA9685 Board


> A ROS node to control servos on a PCA9685 board

The primary purpose of this project is to develop a ROS node capable of controlling servos for steering and throttle, such as would be used by a [Donkey Car](http://www.donkeycar.com/), using the standard Adafruit PCA9685 board, controlled over the I2C bus from a Raspberry Pi.

## Table of Contents

- [Dependencies](#dependencies)
- [Install](#install)
- [Usage](#usage)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Dependencies

ROS, obviously. If you don't want to suffer the pain of trying to build ROS on the standard RPi Linux distros, you might look into [this tasty image](https://downloads.ubiquityrobotics.com/pi.html), with ROS pre-installed, from Ubiquity Robotics.

This project also depends on the [wiringPi](http://wiringpi.com/) library for I2C communication. It may already be present on your Pi. You can learn more about it [here](http://wiringpi.com/download-and-install/).

## Install

On your Pi, once you've verified wiringPi has been installed, create a Catkin workspace directory and clone this repository into `src`.

```
mkdir -p ~/catkin_ws/src
cd src
git clone git@github.com:liamondrop/ros-pca9685-board.git
```

You should now be able to build with Catkin.

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

In the launch file, there is an example servo configuration. This will set the parameters for the steering and throttle servos, as well as the pwm frequency. You may need to change these for your particular setup.

 - channel: This should be a value from 0 to 15, corresponding to the pin on your PCA9685 board connected to the servo you are configuring.
 - center: For the steering servo, this will be the pulse value (somewhere between 0 and 4096) that causes your wheels to point straight ahead. For the throttle, this will be the pulse value at which the throttle is stopped.
 - direction: Either 1 or -1, depending on the servo. Some experimentation is required.
 - range: The difference between the high and low signal. For example, if a signal of 400 gives you full forward throttle and a signal of 300 gives you full reverse throttle, then the range is 100 (and your center value should probably be 350).

One note on throttle servos for RC cars: when switching from forward to reverse, it is actually necessary to send a reverse pulse twice before the servo will respond.

You may also set the PWM Frequency in the configuration. It defaults to 50hz in this project, which is the standard recommended frequency for servos. If you change this value, you will likely need to update your servo configurations as well.

Once you haveLaunch the `pca9685_board` node.

```
roslaunch pca9685_board pca9685.launch
```

The `pca9685_board` node listens to two topics:

 - `/servo_absolute`: this topic accepts a `pca9685_board::Servo` message, which is built by this project. The main purpose of this topic is to send pwm signals to help you fine tune the configuration of your servos. The following messages are an example of using this topic to find the pulse value corresponding to the steering servo's center, i.e. 333.
    ```
    rostopic pub servo_absolute pca9685_board/Servo "{name: steering, value: 300}"
    rostopic pub servo_absolute pca9685_board/Servo "{name: steering, value: 350}"
    rostopic pub servo_absolute pca9685_board/Servo "{name: steering, value: 330}"
    rostopic pub servo_absolute pca9685_board/Servo "{name: steering, value: 335}"
    rostopic pub servo_absolute pca9685_board/Servo "{name: steering, value: 333}"
 - `/servos_drive`: this topic is intended for control and accepts a standard `geometry_msgs::Twist` message, using the `linear.x` value for the throttle and the `angular.z` value for the steering. Note that these values should be between -1.0 and 1.0 inclusive.
    ```
    rostopic pub servos_drive geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: -0.75}}"
    ```

If you have completed the [TeleopTurtle Joystick tutorial](http://wiki.ros.org/joy/Tutorials/WritingTeleopNode), you could control the servos with a joystick by simply relaying the `/turtle1/cmd_vel` topic to the `/servos_drive` topic.

```
rosrun topic_tools relay /turtle1/cmd_vel /servos_drive
```

Note that the angular and linear scales in that project are set to 2, and you will probably want to change them to 1 so the joystick output only ranges from -1.0 to 1.0, as required by this module.

## Maintainers

[@liamondrop](https://github.com/liamondrop)

## Contributing

PRs accepted.

Small note: If editing the README, please conform to the [standard-readme](https://github.com/RichardLitt/standard-readme) specification.

## License

MIT © 2018 Liam Bowers
