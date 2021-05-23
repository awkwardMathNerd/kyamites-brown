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
#include <math.h>

#include <logging/log.h>
#include <drivers/gpio.h>
#include <drivers/shields/crickit.h>

/* Typedefs --------------------------------------------------- */

typedef struct {

    double volt_out;
    double volt_ref;

} methane_sensor_t;

/* Defines ---------------------------------------------------- */

#define PWR_NODE                    DT_ALIAS(pwr)

#define PWR_LABEL                   DT_GPIO_LABEL(PWR_NODE, gpios)
#define PWR_PIN                     DT_GPIO_PIN(PWR_NODE, gpios)
#define PWR_FLAGS                   DT_GPIO_FLAGS(PWR_NODE, gpios)

#define VOLT_OUT_SIGNAL             CRICKIT_SIGNAL1
#define VOLT_REF_SIGNAL             CRICKIT_SIGNAL2
#define PUMP_CHANNEL1               CRICKIT_MOTOR_A1
#define PUMP_CHANNEL2               CRICKIT_MOTOR_A2
#define VALUE_CHANNEL1              CRICKIT_MOTOR_B1
#define VALUE_CHANNEL2              CRICKIT_MOTOR_B2

#define M0                          (-2.969362296f)
#define M1                          (-2.321928095f)
#define M2                          (-2.40942084f)
#define C0                          (3.893867119f)
#define C1                          (3.698970004f)
#define C2                          (3.698970004f)
#define VCC                         (5.0f)

#define BREAK0                      (2.0f)
#define BREAK1                      (1.0f)
#define BREAK2                      (0.75f)

/* Macros ----------------------------------------------------- */

/* Variables -------------------------------------------------- */

/* Prototypes ------------------------------------------------ */

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

int pwr_timer_init(const struct device *dev) {

    int err;

    err = gpio_pin_configure(dev, PWR_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("failed configure pwr pin");
        return;
    }

    return 0;
}

static uint32_t convert_voltage_concentration(methane_sensor_t *sensor) {

    float RsRo = ((VCC / sensor->volt_out) - 1) / ((VCC / sensor->volt_ref) - 1);

    float logRsRo = log10(RsRo);
    float logC = 0.0f;

    if (RsRo > BREAK0) {

        logC = M0 * logRsRo + C0;
    } else if (RsRo > BREAK1) {

        logC = M1 * logRsRo + C1;
    } else {

        logC = M2 * logRsRo + C2;
    }

    float C = pow(10.0f, logC);

    return (uint32_t)C;
}

static float convert_voltage_sample(int16_t raw) {

    double val = (double)((double)(raw) / 310.3f);

    return val;

}

int methane_sample_fetch(const struct device *dev, methane_sensor_t *sensor) {

    int err;
    const struct crickit_api *crickit = dev->api;

    uint16_t val[1];

    err = crickit->analog_read(dev, VOLT_OUT_SIGNAL, val);
    if (err) {
        LOG_ERR("failed fetch methane sensor voltage out (err %d)", err);
        return err;
    }

    sensor->volt_out = convert_voltage_sample(val[0]);

    err = crickit->analog_read(dev, VOLT_REF_SIGNAL, val);
    if (err) {
        LOG_ERR("failed fetch methane sensor voltage ref (err %d)", err);
        return err;
    }

    sensor->volt_ref = convert_voltage_sample(val[0]);

    return 0;
}

int power_off(const struct device *dev) {
    
    int err;

    err = gpio_pin_set(dev, PWR_PIN, 1);
    if (err) {
        LOG_ERR("failed power off the device");
        return err;
    }

    return 0;
}

int turn_pump_on(const struct device *dev) {

    int err;

    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, PUMP_CHANNEL1, 65535);
    if (err) {
        LOG_ERR("failed turn on pump channel 1");
        return err;
    }

    err = crickit->analog_write(dev, PUMP_CHANNEL2, 0);
    if (err) {
        LOG_ERR("failed turn on pump channel 2");
        return err;
    }

    return 0;
}

int turn_pump_off(const struct device *dev) {

    int err;

    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, PUMP_CHANNEL1, 0);
    if (err) {
        LOG_ERR("failed turn off pump channel 1");
        return err;
    }

    err = crickit->analog_write(dev, PUMP_CHANNEL2, 0);
    if (err) {
        LOG_ERR("failed turn off pump channel 2");
        return err;
    }

    return 0;
}

int open_valve(const struct device *dev) {

    int err;

    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, VALUE_CHANNEL1, 65535);
    if (err) {
        LOG_ERR("failed open valve channel 1");
        return err;
    }

    err = crickit->analog_write(dev, VALUE_CHANNEL2, 0);
    if (err) {
        LOG_ERR("failed open valve channel 2");
        return err;
    }

    return 0;
}

int close_valve(const struct device *dev) {

    int err;

    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, VALUE_CHANNEL1, 0);
    if (err) {
        LOG_ERR("failed open valve channel 1");
        return err;
    }

    err = crickit->analog_write(dev, VALUE_CHANNEL2, 0);
    if (err) {
        LOG_ERR("failed open valve channel 2");
        return err;
    }

    return 0;
}

void main(void) {

	int err;

    // methane_sensor_t sensor;

    const struct device *crickit = device_get_binding(DT_LABEL(DT_INST(0, adafruit_crickit)));
    if (!crickit) {
        LOG_ERR("failed get CRICKIT shield binding");
        return;
    }

    k_sleep(K_MSEC(10));

    const struct device *pwr_bus = device_get_binding(PWR_LABEL);
    if (!pwr_bus) {
        LOG_ERR("failed get pwr gpio binding");
        return;
    }

	LOG_INF("Init ok");

    k_sleep(K_MSEC(5000));

    err = power_off(pwr_bus);
    if (err) {
        return;
    }

    // for (;;) {

    //     err = methane_sample_fetch(crickit, &sensor);
    //     if (err) {
    //         LOG_ERR("failed fetch voltage samples");
    //         return;
    //     }

    //     LOG_INF("out:%d ref:%d", (int)(sensor.volt_out * 100), (int)(sensor.volt_ref * 100));

    //     uint32_t ppm = convert_voltage_concentration(&sensor);
    //     LOG_INF("ppm: %d", ppm);

    //     // err = turn_pump_on(crickit);
    //     // if (err) {
    //     //     LOG_ERR("failed turn pump on");
    //     //     return;
    //     // }

    //     // err = open_vent(crickit);
    //     // if (err) {
    //     //     LOG_ERR("failed open vent");
    //     //     return;
    //     // }

    //     // k_sleep(K_MSEC(5000));

    //     // err = turn_pump_off(crickit);
    //     // if (err) {
    //     //     LOG_ERR("failed turn pump off");
    //     //     return;
    //     // }

    //     // err = close_vent(crickit);
    //     // if (err) {
    //     //     LOG_ERR("failed close vent");
    //     //     return;
    //     // }

    //     k_sleep(K_MSEC(5000));

    // }

    // for (;;) {

    //     err = turn_motor_on(crickit);
    //     if (err) {
    //         return;
    //     }

    //     k_sleep(K_MSEC(2500));

    //     err = turn_motor_off(crickit);
    //     if (err) {
    //         return;
    //     }

    //     k_sleep(K_MSEC(2500));

    // }

}
