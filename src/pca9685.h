/*************************************************************************
 * pca9685.h
 *
 **************************************************************************
 */
 
 
#ifdef __cplusplus
extern "C" {
#endif

// Setup a pca9685 at the specific i2c address
extern int pca9685Setup(const int pinBase, const int i2cAddress/* = 0x40*/, float freq/* = 50*/);

// You now have access to the following wiringPi functions:
//
// void pwmWrite (int pin, int value)
//		if value <= 0, set full-off
//		else if value >= 4096, set full-on
//		else set PWM
//
// void digitalWrite (int pin, int value)
// 		if value != 0, set full-on
//		else set full-off
//
// int digitalRead (int pin)
//		read off-register
//		To get PWM: mask with 0xFFF
//		To get full-off bit: mask with 0x1000
//		Note: ALL_LED pin will always return 0
//
// int analogRead (int pin)
//		read on-register
//		To get PWM: mask with 0xFFF
//		To get full-on bit: mask with 0x1000
//		Note: ALL_LED pin will always return 0



// Advanced controls
// You can use the file descriptor returned from the setup function to access the following features directly on each connected pca9685
extern void pca9685PWMFreq(int fd, float freq);
extern void pca9685PWMReset(int fd);
extern void pca9685PWMWrite(int fd, int pin, int on, int off);
extern void pca9685PWMRead(int fd, int pin, int *on, int *off);

extern void pca9685FullOn(int fd, int pin, int tf);
extern void pca9685FullOff(int fd, int pin, int tf);

#ifdef __cplusplus
}
#endif
