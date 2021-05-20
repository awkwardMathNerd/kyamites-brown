/**
***************************************************************
* @file apps/s/inc/hal_crickit.h
* @author Lachlan Smith - s4482220
* @date 08032021
* @brief Practical 1
* REFERENCE: README.md
*************************************************************** */
#ifndef HAL_CRICKIT_H_
#define HAL_CRICKIT_H_

#include <stdint.h>

#define CRICKIT_SIGNAL1                 2
#define CRICKIT_SIGNAL2                 3
#define CRICKIT_SIGNAL3                 40
#define CRICKIT_SIGNAL4                 41
#define CRICKIT_SIGNAL5                 11
#define CRICKIT_SIGNAL6                 10
#define CRICKIT_SIGNAL7                 9
#define CRICKIT_SIGNAL8                 8

#define CRICKIT_SERVO4                  14
#define CRICKIT_SERVO3                  15
#define CRICKIT_SERVO2                  16
#define CRICKIT_SERVO1                  17
#define CRICKIT_MOTOR_A1                22
#define CRICKIT_MOTOR_A2                23
#define CRICKIT_MOTOR_B1                19
#define CRICKIT_MOTOR_B2                18
#define CRICKIT_DRIVE1                  13
#define CRICKIT_DRIVE2                  12
#define CRICKIT_DRIVE3                  43
#define CRICKIT_DRIVE4                  42

#ifdef CONFIG_SHIELD_CRICKIT_TOUCH
#define CRICKIT_TOUCH1                  0
#define CRICKIT_TOUCH2                  1
#define CRICKIT_TOUCH3                  2
#define CRICKIT_TOUCH4                  3
#endif

#define CRICKIT_DUTY_CYCLE_OFF 0
#define CRICKIT_DUTY_CYCLE_MAX 65535

struct crickit_data {
    const struct device *i2c;
};

struct crickit_cfg {
    char *dev_name;
    uint16_t dev_addr;
};

struct crickit_api {
	int (*analog_write)(const struct device *dev, uint8_t pin, uint16_t value);
    int (*analog_read)(const struct device *dev, uint8_t pin, uint16_t *value);
    int (*pwm_freq_set)(const struct device *dev, uint8_t pin, uint16_t freq);
};

#endif  // HAL_CRICKIT_H_
