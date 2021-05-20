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

#include <usb/usb_device.h>
#include <logging/log.h>

#include <drivers/shields/crickit.h>

/* Typedefs --------------------------------------------------- */

typedef struct __attribute__((__packed__, __aligned__(4), scalar_storage_order("little-endian"))) {

    uint16_t volt_out;
    uint16_t volt_ref;

} methane_sensor_t;

/* Defines ---------------------------------------------------- */

#define VOLT_OUT_SIGNAL             CRICKIT_SIGNAL1
#define VOLT_REF_SIGNAL             CRICKIT_SIGNAL2
#define POWER_OFF_SIGNAL            CRICKIT_SIGNAL3

/* Macros ----------------------------------------------------- */

/* Variables -------------------------------------------------- */

/* Prototypes ------------------------------------------------ */

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

int methane_sample_fetch(const struct device *dev, methane_sensor_t *sensor) {

    int err;
    const struct crickit_api *crickit = dev->api;

    uint16_t val[1];

    err = crickit->analog_read(dev, VOLT_OUT_SIGNAL, val);
    if (err) {
        LOG_ERR("failed fetch methane sensor voltage out (err %d)", err);
        return err;
    }

    sensor->volt_out = val[0];

    err = crickit->analog_read(dev, VOLT_REF_SIGNAL, val);
    if (err) {
        LOG_ERR("failed fetch methane sensor voltage ref (err %d)", err);
        return err;
    }

    sensor->volt_ref = val[0];

    return 0;
}

int power_off(const struct device *dev) {

    int err;
    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, POWER_OFF_SIGNAL, 2048);
    if (err) {
        LOG_ERR("failed analogue write power off (err %d)", err);
        return -EIO;
    }

    return 0;
}

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

    const struct device *crickit = device_get_binding(DT_LABEL(DT_INST(0, adafruit_crickit)));
    if (!crickit) {
        LOG_ERR("failed get CRICKIT shield binding");
        return;
    }

	LOG_INF("Init ok");

    methane_sensor_t sensor;

    for (;;) {

        err = methane_sample_fetch(crickit, &sensor);
        if (err) {
            goto off;
        }

        off:do {

            err = power_off(crickit);

        } while (!err);

        k_sleep(K_MSEC(1500));
    }

}
