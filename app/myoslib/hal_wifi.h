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


#ifndef HAL_WIFI_H
#define HAL_WIFI_H

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(app);

#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_event.h>

// LEDs for indicating the status of the WiFi
#define GREEN_LED_NODE    DT_ALIAS(led2)
#define BLUE_LED_NODE     DT_ALIAS(led3)

#define GREEN_LED         DT_GPIO_LABEL(GREEN_LED_NODE, gpios)
#define GREEN_LED_PIN     DT_GPIO_PIN(GREEN_LED_NODE, gpios)
#define GREEN_LED_FLAGS   DT_GPIO_FLAGS(GREEN_LED_NODE, gpios)

#define BLUE_LED          DT_GPIO_LABEL(BLUE_LED_NODE, gpios)
#define BLUE_LED_PIN      DT_GPIO_PIN(BLUE_LED_NODE, gpios)
#define BLUE_LED_FLAGS    DT_GPIO_FLAGS(BLUE_LED_NODE, gpios)

/* WiFi connection parameters */
#define WIFI_SSID "infrastructure"
#define WIFI_PSK "WaESjCqZt26A"

#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface);

extern void hal_wifi_init(void);

#endif