/**
***************************************************************
* @file apps/p3/MOBILE/src/main.c
* @author Lachlan Smith - s4482220
* @date 08032021
* @brief Practical 3
*************************************************************** */

/* Includes --------------------------------------------------- */

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>

/* Typedefs --------------------------------------------------- */

/* Defines ---------------------------------------------------- */

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <usb/usb_device.h>

#include <settings/settings.h>

#include <drivers/shields/crickit.h>

/* Macros ----------------------------------------------------- */

/* Variables -------------------------------------------------- */

/* Prototypes ------------------------------------------------ */

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

void main(void) {

	int err;

	const struct device *usb = device_get_binding("CDC_ACM_0");
    if (!usb) {
		LOG_ERR("failed get CDC ACM device");
        return;
    }

    err = usb_enable(NULL);
    if (err) {
        LOG_ERR("failed enable usb (err %d)", err);
        return;
    } 

    const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, adafruit_crickit)));
    if (!dev) {
        LOG_ERR("failed get CRICKIT shield binding");
        return;
    }

	LOG_INF("Init ok");

    for (;;) {

        const struct crickit_api *crickit = dev->api;

        crickit->analog_write(dev, CRICKIT_MOTOR_A1, 2048);

        k_sleep(K_MSEC(500));
    }

}
