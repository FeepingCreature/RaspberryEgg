#ifndef RASPBERRYEGG_CONFIG_H
#define RASPBERRYEGG_CONFIG_H

#define SERVO_PIN 26
// lower range of servo movement
#define SERVO_LOW 0.16
// upper range of servo movement
#define SERVO_HIGH 0.4
// servo pwm length in seconds
#define SERVO_PWM_LENGTH (1.0/50.0)
// servo pwm cycle fraction for lowest servo position
#define SERVO_PWM_LOW (0.5/20.0)
// servo pwm cycle fraction for highest servo position
#define SERVO_PWM_HIGH (3.0/20.0)

// base factor for pwm to ensure steppers don't get too much current
#define BASE_PWM_FACTOR 0.55

// egg rotation stepper in1/in2 (winding 1)
#define STEPPER_EGG_PIN1 4
#define STEPPER_EGG_PIN2 17
// egg rotation stepper in3/in4 (winding 2)
#define STEPPER_EGG_PIN3 18
#define STEPPER_EGG_PIN4 27

// pen rotation stepper in1/in2 (winding 1)
#define STEPPER_PEN_PIN1 22
#define STEPPER_PEN_PIN2 23
// pen rotation stepper in3/in4 (winding 2)
#define STEPPER_PEN_PIN3 24
#define STEPPER_PEN_PIN4 25

#define US_PER_PWM 40.0
#define CALIBRATION_CYCLES (1024 * 1024 * 8)

#endif
