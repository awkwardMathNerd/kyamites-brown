/**
************************************************************
* @file     myoslib/hal_wifi.h   
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

const struct device *hal_wifi_green_led_dev = NULL;
const struct device *hal_wifi_blue_led_dev = NULL;

static struct net_mgmt_event_callback hal_wifi_mgmt_cb;

volatile uint8_t hal_wifi_is_connected = 0;

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

	switch (mgmt_event) {
		case NET_EVENT_WIFI_CONNECT_RESULT:

			if (!status->status) {

				gpio_pin_set(hal_wifi_green_led_dev, HAL_WIFI_GREEN_LED_PIN, 1);
				gpio_pin_set(hal_wifi_blue_led_dev, HAL_WIFI_BLUE_LED_PIN, 0);
				hal_wifi_is_connected = 1;
			}

			LOG_INF("%d", status->status);
			break;
		case NET_EVENT_WIFI_DISCONNECT_RESULT:
			
			/* TODO: SEND DONE HERE */
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
	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;
	
	cnx_params.channel = WIFI_CHANNEL_ANY;
	cnx_params.ssid = HAL_WIFI_SSID;
	cnx_params.ssid_length = strlen(HAL_WIFI_SSID);
	cnx_params.security = WIFI_SECURITY_TYPE_PSK;
	cnx_params.psk = HAL_WIFI_PSK;
	cnx_params.psk_length = strlen(HAL_WIFI_PSK);

	/* Try the connection */
	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
		     &cnx_params, sizeof(struct wifi_connect_req_params))) {
		
		return;
	}

	while (hal_wifi_is_connected == 0) {
		k_msleep(100);
	}

	LOG_INF("WiFi is connected");
}

