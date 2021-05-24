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

#include "hal_wifi.h"
#include "hal_mqtt.h"

/* Typedefs --------------------------------------------------- */

typedef struct {

    double volt_out;
    double volt_ref;

} methane_sensor;

/* Defines ---------------------------------------------------- */

#define PPM_THRES                   30

#define PWR_NODE                    DT_ALIAS(pwr)
#define PWR_LABEL                   DT_GPIO_LABEL(PWR_NODE, gpios)
#define PWR_PIN                     DT_GPIO_PIN(PWR_NODE, gpios)
#define PWR_FLAGS                   DT_GPIO_FLAGS(PWR_NODE, gpios)

#define VOLT_OUT_SIGNAL             CRICKIT_SIGNAL1
#define VOLT_REF_SIGNAL             CRICKIT_SIGNAL2
#define PUMP_CHANNEL1               CRICKIT_MOTOR_A1
#define PUMP_CHANNEL2               CRICKIT_MOTOR_A2
#define VALVE_CHANNEL1              CRICKIT_MOTOR_B1
#define VALVE_CHANNEL2              CRICKIT_MOTOR_B2

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

/**
 * @brief Configure the power timer pin
 *
 * @param dev Pointer to the power timer gpio bus
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int pwr_timer_init(const struct device *dev) {

    return gpio_pin_configure(dev, PWR_PIN, GPIO_OUTPUT_INACTIVE);
}

/**
 * @brief Set the power timer pin high to turn of the device
 *
 * @param dev Pointer to the power timer gpio bus
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int power_off(const struct device *dev) {

    return gpio_pin_set(dev, PWR_PIN, 1);
}

/**
 * @brief Set the power timer pin high to turn of the device
 *
 * @param sensor Methane sensor voltage out and ref structure
 *
 * @retval parts per million methane measurement in the chamber
 */
static uint32_t convert_voltage_concentration(methane_sensor *sensor) {

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


/**
 * @brief Convert the raw sensor voltage to a value
 *
 * @param raw raw sample voltage value from the sensor
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static double convert_voltage_sample(int16_t raw) {

    double val = (double)((double)(raw) / 310.3f);

    return val;

}

/**
 * @brief Take voltage out and ref samples from the methane sensor
 *
 * @param dev Pointer to the crickit device structure
 * @param sensor Methane sensor voltage out and ref structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int methane_sample_fetch(const struct device *dev, methane_sensor *sensor) {

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

/**
 * @brief Turn the pump on connected to the CRICKIT board
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
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

/**
 * @brief Turn the pump off connected to the CRICKIT board
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
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

/**
 * @brief Open the valve connected to the CRICKIT board
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int open_valve(const struct device *dev) {

    int err;

    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, VALVE_CHANNEL1, 65535);
    if (err) {
        LOG_ERR("failed open valve channel 1");
        return err;
    }

    err = crickit->analog_write(dev, VALVE_CHANNEL2, 0);
    if (err) {
        LOG_ERR("failed open valve channel 2");
        return err;
    }

    return 0;
}

/**
 * @brief Close the valve connected to the CRICKIT board
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int close_valve(const struct device *dev) {

    int err;

    const struct crickit_api *crickit = dev->api;

    err = crickit->analog_write(dev, VALVE_CHANNEL1, 0);
    if (err) {
        LOG_ERR("failed open valve channel 1");
        return err;
    }

    err = crickit->analog_write(dev, VALVE_CHANNEL2, 0);
    if (err) {
        LOG_ERR("failed open valve channel 2");
        return err;
    }

    return 0;
}

/**
 * @brief Main methane ppm publishing device application
 */
void main(void) {

    int err;
    methane_sensor sensor;

    const struct device *pwr_bus = device_get_binding(PWR_LABEL);
    if (!pwr_bus) {
        LOG_ERR("failed configure power timer");
        return;
    }

    err = pwr_timer_init(pwr_bus);
    if (err) {
        return;
    }

    const struct device *crickit = device_get_binding(DT_LABEL(DT_INST(0, adafruit_crickit)));
    if (!crickit) {
        LOG_ERR("failed to find the CRICKIT board");
        return;
    }

    hal_wifi_init();
	hal_mqtt_init();

    LOG_INF("Init ok");

    k_sleep(K_MSEC(250));

    err = methane_sample_fetch(crickit, &sensor);
    if (err) {
        goto OFF;
    }

    uint32_t ppm = convert_voltage_concentration(&sensor);

    LOG_INF("ppm: %d", ppm);

    hal_mqtt_payload_update(sensor.volt_ref, ppm);
    hal_mqtt_publish();

    /* Do we need to vent the chamber */
    if (ppm > PPM_THRES) {

        /* Pump in fresh air */
        err = turn_pump_on(crickit);
        if (err) {
            goto OFF;
        }

        /* Allow air to circulate out */
        err = open_valve(crickit);
        if (err) {
            goto OFF;
        }

        /* Spend thirty seconds venting */
        k_sleep(K_MSEC(30000));

        err = turn_pump_off(crickit);
        if (err) {
            goto OFF;
        }

        err = close_valve(crickit);
        if (err) {
            goto OFF;
        }
    }

    k_sleep(K_MSEC(250));

    /* write high to the power timer regulator */
    OFF:power_off(pwr_bus);

    /* Should never get here */
    LOG_ERR("failed power off");
}

