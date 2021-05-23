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

const struct device *green_led_dev = NULL;
const struct device *blue_led_dev = NULL;

static struct net_mgmt_event_callback wifi_mgmt_cb;

volatile uint8_t isConnected = 0;

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
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface) {
                        
	/* Get status out of callback */
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	switch (mgmt_event) {
		case NET_EVENT_WIFI_CONNECT_RESULT:

			if (!status->status) {

				gpio_pin_set(green_led_dev, GREEN_LED_PIN, 1);
				gpio_pin_set(blue_led_dev, BLUE_LED_PIN, 0);
				isConnected = 1;
			}
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
    green_led_dev = device_get_binding(GREEN_LED);
    if (green_led_dev == NULL) {

        return;
    }

    blue_led_dev = device_get_binding(BLUE_LED);
    if (blue_led_dev == NULL) {

        return;
    }

    int ret;

    ret = gpio_pin_configure(green_led_dev, GREEN_LED_PIN, 
            GPIO_OUTPUT_ACTIVE | GREEN_LED_FLAGS);
    if (ret < 0) {
        return;
    }

    ret = gpio_pin_configure(blue_led_dev, BLUE_LED_PIN, 
            GPIO_OUTPUT_ACTIVE | BLUE_LED_FLAGS);
    if (ret < 0) {
        return;
    }

    /* Turn the blue LED on and green LED off*/
    gpio_pin_set(blue_led_dev, BLUE_LED_PIN, 1);
    gpio_pin_set(green_led_dev, GREEN_LED_PIN, 0);

    /* Send the connect for the stuff */
	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;
	
	cnx_params.channel = WIFI_CHANNEL_ANY;
	cnx_params.ssid = WIFI_SSID;
	cnx_params.ssid_length = strlen(WIFI_SSID);
	cnx_params.security = WIFI_SECURITY_TYPE_PSK;
	cnx_params.psk = WIFI_PSK;
	cnx_params.psk_length = strlen(WIFI_PSK);

	/* Try the connection */
	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
		     &cnx_params, sizeof(struct wifi_connect_req_params))) {
		
		return;
	}

	while (isConnected == 0) {
		k_msleep(100);
	}

	LOG_INF("WiFi is connected");
}

