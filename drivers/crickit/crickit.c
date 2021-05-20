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

#include <drivers/shields/crickit.h>

/* Typedefs --------------------------------------------------- */

/* Defines ---------------------------------------------------- */

#define DT_DRV_COMPAT           adafruit_crickit

#define CRICKIT_NUM_PWM         12
#define CRICKIT_NUM_ADC         8

#define CRICKIT_HW_ID_CODE      0x55

/* Macros ----------------------------------------------------- */

enum {
  CRICKIT_STATUS_BASE = 0x00,
  CRICKIT_GPIO_BASE = 0x01,
  CRICKIT_SERCOM0_BASE = 0x02,
  CRICKIT_TIMER_BASE = 0x08,
  CRICKIT_ADC_BASE = 0x09,
  CRICKIT_DAC_BASE = 0x0A,
  CRICKIT_INTERRUPT_BASE = 0x0B,
  CRICKIT_DAP_BASE = 0x0C,
  CRICKIT_EEPROM_BASE = 0x0D,
  CRICKIT_NEOPIXEL_BASE = 0x0E,
  CRICKIT_TOUCH_BASE = 0x0F,
  CRICKIT_KEYPAD_BASE = 0x10,
  CRICKIT_ENCODER_BASE = 0x11,
};

enum {
  CRICKIT_STATUS_HW_ID = 0x01,
  CRICKIT_STATUS_VERSION = 0x02,
  CRICKIT_STATUS_OPTIONS = 0x03,
  CRICKIT_STATUS_TEMP = 0x04,
  CRICKIT_STATUS_SWRST = 0x7F,
};

#ifdef CONFIG_SHIELD_CRICKIT_GPIO
enum {
  CRICKIT_GPIO_DIRSET_BULK = 0x02,
  CRICKIT_GPIO_DIRCLR_BULK = 0x03,
  CRICKIT_GPIO_BULK = 0x04,
  CRICKIT_GPIO_BULK_SET = 0x05,
  CRICKIT_GPIO_BULK_CLR = 0x06,
  CRICKIT_GPIO_BULK_TOGGLE = 0x07,
  CRICKIT_GPIO_INTENSET = 0x08,
  CRICKIT_GPIO_INTENCLR = 0x09,
  CRICKIT_GPIO_INTFLAG = 0x0A,
  CRICKIT_GPIO_PULLENSET = 0x0B,
  CRICKIT_GPIO_PULLENCLR = 0x0C,
};
#endif

// #ifdef CONFIG_SHIELD_CRICKIT_TIMER
enum {
  CRICKIT_TIMER_STATUS = 0x00,
  CRICKIT_TIMER_PWM = 0x01,
  CRICKIT_TIMER_FREQ = 0x02,
};
// #endif

// #ifdef CONFIG_SHIELD_CRICKIT_ADC
enum {
  CRICKIT_ADC_STATUS = 0x00,
  CRICKIT_ADC_INTEN = 0x02,
  CRICKIT_ADC_INTENCLR = 0x03,
  CRICKIT_ADC_WINMODE = 0x04,
  CRICKIT_ADC_WINTHRESH = 0x05,
  CRICKIT_ADC_CHANNEL_OFFSET = 0x07,
};
// #endif

#ifdef CONFIG_SHIELD_CRICKIT_SERCOM
enum {
  CRICKIT_SERCOM_STATUS = 0x00,
  CRICKIT_SERCOM_INTEN = 0x02,
  CRICKIT_SERCOM_INTENCLR = 0x03,
  CRICKIT_SERCOM_BAUD = 0x04,
  CRICKIT_SERCOM_DATA = 0x05,
};
#endif

#ifdef CONFIG_SHIELD_CRICKIT_NEOPIXEL
enum {
  CRICKIT_NEOPIXEL_STATUS = 0x00,
  CRICKIT_NEOPIXEL_PIN = 0x01,
  CRICKIT_NEOPIXEL_SPEED = 0x02,
  CRICKIT_NEOPIXEL_BUF_LENGTH = 0x03,
  CRICKIT_NEOPIXEL_BUF = 0x04,
  CRICKIT_NEOPIXEL_SHOW = 0x05,
};
#endif

#ifdef CONFIG_SHIELD_CRICKIT_TOUCH
enum {
  CRICKIT_TOUCH_CHANNEL_OFFSET = 0x10,
};
#endif

#ifdef CONFIG_SHIELD_CRICKIT_KEYPAD
enum {
  CRICKIT_KEYPAD_STATUS = 0x00,
  CRICKIT_KEYPAD_EVENT = 0x01,
  CRICKIT_KEYPAD_INTENSET = 0x02,
  CRICKIT_KEYPAD_INTENCLR = 0x03,
  CRICKIT_KEYPAD_COUNT = 0x04,
  CRICKIT_KEYPAD_FIFO = 0x10,
};

enum {
  CRICKIT_KEYPAD_EDGE_HIGH = 0,
  CRICKIT_KEYPAD_EDGE_LOW,
  CRICKIT_KEYPAD_EDGE_FALLING,
  CRICKIT_KEYPAD_EDGE_RISING,
};
#endif

#ifdef CONFIG_SHIELD_CRICKIT_ENCODER
enum {
  CRICKIT_ENCODER_STATUS = 0x00,
  CRICKIT_ENCODER_INTENSET = 0x10,
  CRICKIT_ENCODER_INTENCLR = 0x20,
  CRICKIT_ENCODER_POSITION = 0x30,
  CRICKIT_ENCODER_DELTA = 0x40,
};
#endif

/* Variables -------------------------------------------------- */


static const uint8_t crickit_pwms[CRICKIT_NUM_PWM] = {
    CRICKIT_SERVO4,
    CRICKIT_SERVO3,   
    CRICKIT_SERVO2,   
    CRICKIT_SERVO1,
    CRICKIT_MOTOR_B1, 
    CRICKIT_MOTOR_B2, 
    CRICKIT_MOTOR_A1, 
    CRICKIT_MOTOR_A2,
    CRICKIT_DRIVE4,   
    CRICKIT_DRIVE3,   
    CRICKIT_DRIVE2,   
    CRICKIT_DRIVE1
};

static const uint8_t crickit_adc[CRICKIT_NUM_ADC] = {
    CRICKIT_SIGNAL1, 
    CRICKIT_SIGNAL2, 
    CRICKIT_SIGNAL3, 
    CRICKIT_SIGNAL4,
    CRICKIT_SIGNAL5, 
    CRICKIT_SIGNAL6, 
    CRICKIT_SIGNAL7, 
    CRICKIT_SIGNAL8
};

/* Prototypes ------------------------------------------------- */ 

LOG_MODULE_REGISTER(crickit, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief i2c write function specific to the CRICKIT macros
 *
 * This routine performs two i2c writes, one specifying the 2 byte register 
 * address the other buffer being written
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int crickit_i2c_write(const struct device *dev, uint8_t base, uint8_t addr, uint8_t *buf, size_t len) {

    const struct crickit_cfg * const cfg = dev->config;
	struct crickit_data *data = dev->data;

    struct i2c_msg msg[2];
    uint8_t reg_addr[2] = {base, addr};

	msg[0].buf = (uint8_t *)reg_addr;
	msg[0].len = 2;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = len;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(data->i2c, msg, 2, cfg->dev_addr);
}

/**
 * @brief i2c read function specific to the CRICKIT macros
 *
 * This routine performs i2c write immediately follow an i2c read, 
 * one specifying the 2 byte register address the other buffer being read
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int crickit_i2c_read(const struct device *dev, uint8_t base, uint8_t addr, uint8_t *buf, size_t len) {

    const struct crickit_cfg * const cfg = dev->config;
	struct crickit_data *data = dev->data;

    struct i2c_msg msg[2];
    uint8_t reg_addr[2] = {base, addr};

	msg[0].buf = (uint8_t *)reg_addr;
	msg[0].len = 2;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = len;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(data->i2c, msg, 2, cfg->dev_addr);
}

/**
 * @brief Writes to an analogue pin on the CRICKIT board
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -EINVAL Invalid
 */
int crickit_analog_write(const struct device *dev, uint8_t pin, uint16_t value) {

    int err;

    int8_t p = -1;
    for (int i = 0; i < CRICKIT_NUM_PWM; i++) {
        if (crickit_pwms[i] == pin) {
            p = i;
            break;
        }
    }

    if (p > -1) {

        uint8_t cmd[3] = {
            (uint8_t)p, (uint8_t)(value >> 8), (uint8_t)value
        };

        err = crickit_i2c_write(dev, CRICKIT_TIMER_BASE, CRICKIT_TIMER_PWM, cmd, sizeof(cmd));
        if (err) {
            LOG_ERR("failed analogue i2c write (err %d)", err);
            return -EIO;
        }

    } else {
        LOG_ERR("tried to write an invalid analogue pin");
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Reads from a analogue pin on the CRICKIT board
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -EINVAL Invalid
 */
int crickit_analog_read(const struct device *dev, uint8_t pin, uint16_t *value) {

    int err;

    uint8_t val[2];
    int8_t p = -1;
    for (uint8_t i = 0; i < CRICKIT_NUM_ADC; i++) {

        if (crickit_adc[i] == pin) {
            p = i;
            break;
        }
    }

    if (p > -1) {

        err = crickit_i2c_read(dev, CRICKIT_ADC_BASE, CRICKIT_ADC_CHANNEL_OFFSET + p, val, sizeof(val));
        if (err) {
            LOG_ERR("failed analogue i2c read (err %d)", err);
            return -EIO;
        }

        *value = (uint16_t)((val[0] << 8) | val[1]);
        k_sleep(K_MSEC(1));

    } else {
        LOG_ERR("tried to read an invalid analogue pin");
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Set the PWM frequency of an analogue pin
 *
 * @param dev Pointer to the crickit device structure
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -EINVAL Invalid
 */
int crickit_pwm_frequency_set(const struct device *dev, uint8_t pin, uint16_t freq) {

    int err;

    int8_t p = -1;
    for (uint8_t i = 0; i < CRICKIT_NUM_PWM; i++) {

        if (crickit_pwms[i] == pin) {
            p = i;
            break;
        }
    }

    if (p > -1) {

        uint8_t cmd[3] = {
            (uint8_t)p, (uint8_t)(freq >> 8), (uint8_t)freq
        };

        err = crickit_i2c_write(dev, CRICKIT_TIMER_BASE, CRICKIT_TIMER_FREQ, cmd, sizeof(cmd));
        if (err) {
            LOG_ERR("failed analogue i2c write (err %d)", err);
            return -EIO;
        }
    } else {
        LOG_ERR("tried to set an invalid analogue pin");
        return -EINVAL;
    }

    
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

    int err;

	const struct crickit_cfg * const cfg = dev->config;
	struct crickit_data *data = dev->data;
    // const struct crickit_api *api = dev->api;

	data->i2c = device_get_binding(cfg->dev_name);
	if (data->i2c == NULL) {
        LOG_DBG("failed to get I2C bus binding");
		return -EINVAL;
	}

    uint8_t id;
    err = crickit_i2c_read(dev, CRICKIT_STATUS_BASE, CRICKIT_STATUS_HW_ID, &id, 1);
    if (err) {
        LOG_ERR("failed read crickit shield SAMD21 device id (err %d)", err);
        return err;
    }

    if (id != CRICKIT_HW_ID_CODE) {
        LOG_ERR("failed got wrong HW id code (id %x)", id);
        return -EIO;
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
    .analog_write = crickit_analog_write,
    .analog_read = crickit_analog_read,
    .pwm_freq_set = crickit_pwm_frequency_set,
};

DEVICE_DT_INST_DEFINE(0,					                \
            crickit_init,				                    \
            device_pm_control_nop,			                \
            &crickit_data,			                        \
            &crickit_config,			                    \
            POST_KERNEL,				                    \
            CONFIG_SENSOR_INIT_PRIORITY,		            \
            &crickit_api);
