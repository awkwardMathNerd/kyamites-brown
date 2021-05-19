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

struct crickit_data {
    const struct device *i2c;

    uint16_t ppm;
    int16_t volts;
};

struct crickit_cfg {
    char *dev_name;
    uint16_t dev_addr;
};

struct crickit_api {
	int (*vent)(const struct device *dev);
    int (*sample)(const struct device *dev);
    
};

#endif  // HAL_CRICKIT_H_
