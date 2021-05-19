/**
***************************************************************
* @file apps/p2/SCU/src/main.c
* @author Lachlan Smith - s4482220
* @date 08032021
* @brief Practical 1
*************************************************************** */

/* Includes --------------------------------------------------- */

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <drivers/i2c.h>

#include "hal_crickit.h"

/* Typedefs --------------------------------------------------- */

/* Defines ---------------------------------------------------- */

#define DT_DRV_COMPAT adafruit_crickit

/* Macros ----------------------------------------------------- */

/* Variables -------------------------------------------------- */

/* Prototypes ------------------------------------------------- */ 

LOG_MODULE_REGISTER(crickit, CONFIG_LOG_DEFAULT_LEVEL);

int hal_crickit_vent(const struct device *dev) {

    // const struct crickit_cfg * const cfg = dev->config;
	// struct crickit_data *data = dev->data;
    // const struct crickit_api *api = dev->api;

    return 0;
}

int hal_crickit_sample(const struct device *dev) {

    // const struct crickit_cfg * const cfg = dev->config;
	// struct crickit_data *data = dev->data;
    // const struct crickit_api *api = dev->api;

    return 0;
}

/**
 * @brief Initialise the crickit driver
 *
 * This routine sets driver data structures, identifies the i2c device 
 * and initialise the SAM09 chip
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static int crickit_init(const struct device *dev) {

	const struct crickit_cfg * const cfg = dev->config;
	struct crickit_data *data = dev->data;
    // const struct crickit_api *api = dev->api;

	data->i2c = device_get_binding(cfg->dev_name);
	if (data->i2c == NULL) {
        LOG_DBG("failed to get I2C bus binding");
		return -EINVAL;
	}

    LOG_INF("Init ok");

	return 0;
}

/**
 * @brief crickit data structure
 */
static struct crickit_data crickit_data;

/**
 * @brief crickit configuration structure
 */
static const struct crickit_cfg crickit_config = {
    .dev_name = DT_INST_BUS_LABEL(0),
    .dev_addr = DT_INST_REG_ADDR(0),
};

static const struct crickit_api crickit_api = {
    .vent = hal_crickit_vent,
    .sample = hal_crickit_sample,
};

DEVICE_DT_INST_DEFINE(0,					                \
            crickit_init,				                    \
            device_pm_control_nop,			                \
            &crickit_data,			                        \
            &crickit_config,			                    \
            POST_KERNEL,				                    \
            CONFIG_SENSOR_INIT_PRIORITY,		            \
            &crickit_api);
