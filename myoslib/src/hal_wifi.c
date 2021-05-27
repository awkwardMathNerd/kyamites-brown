/**
************************************************************
* @file     myoslib/hal_wifi.c   
* @author   Brendon Duncan 44826482
* @date     23/05/2021
* @brief    Header file for MQTT functionality
* REFERNCE:      
************************************************************
* EXTERNAL FUNCTIONS
************************************************************
* wifi_mgmt_event_handler() - Callback for handling wifi connect
************************************************************
* INTERNAL FUNCTIONS
************************************************************
* hal_wifi_init() - Initialise wifi LEDs and connection
************************************************************
*/

#include "hal_wifi.h"

LOG_MODULE_REGISTER(hal_wifi);

/* LED structures */
const struct device *hal_wifi_green_led_dev = NULL;
const struct device *hal_wifi_blue_led_dev = NULL;

/* Callback for the wifi handler */
static struct net_mgmt_event_callback hal_wifi_mgmt_cb;

/* Flag for whether or not the wifi is connected */
//volatile uint8_t hal_wifi_is_connected = 0;

/* Semaphore to signal for connection and to retry connection */
K_SEM_DEFINE(hal_wifi_is_connected, 0, 1);
K_SEM_DEFINE(hal_wifi_retry_connection, 0, 1);

/**
 * Send the Wifi connect message
 * @param None
 * @ret None
 */
void hal_wifi_perform_connect(void) {

	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;
	
	/* Set our connection parameters */
	cnx_params.channel = WIFI_CHANNEL_ANY;
	cnx_params.ssid = HAL_WIFI_SSID;
	cnx_params.ssid_length = strlen(HAL_WIFI_SSID);
	cnx_params.security = WIFI_SECURITY_TYPE_PSK;
	cnx_params.psk = HAL_WIFI_PSK;
	cnx_params.psk_length = strlen(HAL_WIFI_PSK);

	/* Try the connection */
	int err = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params, sizeof(struct wifi_connect_req_params));
	LOG_INF("Wifi request: %d", err);
}

/**
 * @brief Callback function for WiFi
 *
 * Handler function for all WiFi callbacks, sets LED pins accordingly to 
 * allow for operation diagnosis
 *
 * @param cb Callback struct pointer
 * @param mgmt_event mgmt_event integer 
 * @param iface Struct pointer for the net_if
 *
 * @retval None
 */
static void hal_wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface) {
                        
	/* Get status out of callback */
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	/* Switch which wifi event we just received */
	switch (mgmt_event) {
		case NET_EVENT_WIFI_CONNECT_RESULT:

			/* Check the status */
			if (!status->status) {

				gpio_pin_set(hal_wifi_green_led_dev, HAL_WIFI_GREEN_LED_PIN, 1);
				gpio_pin_set(hal_wifi_blue_led_dev, HAL_WIFI_BLUE_LED_PIN, 0);
				k_sem_give(&hal_wifi_is_connected);
			} else {

				/* Rety the connect message */
				k_sem_give(&hal_wifi_retry_connection);
			}

			LOG_INF("%d", status->status);
			break;
		case NET_EVENT_WIFI_DISCONNECT_RESULT:
			break;
		default:
			break;
	}
}

/**
 * @brief Initialise the wifi 
 *
 * Initialise the Particle Argon LEDs and establish the WiFi connection
 * to the university industrial WiFi
 *
 * @param None
 *
 * @retval None
 */
void hal_wifi_init(void) {

    /* Initialize the LEDs */
    hal_wifi_green_led_dev = device_get_binding(HAL_WIFI_GREEN_LED);
    if (hal_wifi_green_led_dev == NULL) {

        return;
    }

    hal_wifi_blue_led_dev = device_get_binding(HAL_WIFI_BLUE_LED);
    if (hal_wifi_blue_led_dev == NULL) {

        return;
    }

    int ret;

    ret = gpio_pin_configure(hal_wifi_green_led_dev, HAL_WIFI_GREEN_LED_PIN, 
            GPIO_OUTPUT_ACTIVE | HAL_WIFI_GREEN_LED_FLAGS);
    if (ret < 0) {
        return;
    }

    ret = gpio_pin_configure(hal_wifi_blue_led_dev, HAL_WIFI_BLUE_LED_PIN, 
            GPIO_OUTPUT_ACTIVE | HAL_WIFI_BLUE_LED_FLAGS);
    if (ret < 0) {
        return;
    }

    /* Turn the blue LED on and green LED off*/
    gpio_pin_set(hal_wifi_blue_led_dev, HAL_WIFI_BLUE_LED_PIN, 1);
    gpio_pin_set(hal_wifi_green_led_dev, HAL_WIFI_GREEN_LED_PIN, 0);

	/* Initialize the callback */
	net_mgmt_init_event_callback(&hal_wifi_mgmt_cb,
				     hal_wifi_mgmt_event_handler,
				     HAL_WIFI_MGMT_EVENTS);

	net_mgmt_add_event_callback(&hal_wifi_mgmt_cb);

    /* Send the connect for the stuff */
	hal_wifi_perform_connect();

	/* Wait until we are connected to the WiFi */
	//while (hal_wifi_is_connected == 0) {
	while (k_sem_take(&hal_wifi_is_connected, K_MSEC(100)) != 0) {

		/* Check if we need to retry the connection */
		if (k_sem_take(&hal_wifi_retry_connection, K_NO_WAIT) == 0) {

			/* Wait some time and retry the connection */
			k_msleep(100);
			hal_wifi_perform_connect();
		}
	}

	LOG_INF("WiFi is connected");
}

