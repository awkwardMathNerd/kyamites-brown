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

#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_event.h>

// LEDs for indicating the status of the WiFi
#define HAL_WIFI_GREEN_LED_NODE    DT_ALIAS(led2)
#define HAL_WIFI_BLUE_LED_NODE     DT_ALIAS(led3)

#define HAL_WIFI_GREEN_LED         DT_GPIO_LABEL(HAL_WIFI_GREEN_LED_NODE, gpios)
#define HAL_WIFI_GREEN_LED_PIN     DT_GPIO_PIN(HAL_WIFI_GREEN_LED_NODE, gpios)
#define HAL_WIFI_GREEN_LED_FLAGS   DT_GPIO_FLAGS(HAL_WIFI_GREEN_LED_NODE, gpios)

#define HAL_WIFI_BLUE_LED          DT_GPIO_LABEL(HAL_WIFI_BLUE_LED_NODE, gpios)
#define HAL_WIFI_BLUE_LED_PIN      DT_GPIO_PIN(HAL_WIFI_BLUE_LED_NODE, gpios)
#define HAL_WIFI_BLUE_LED_FLAGS    DT_GPIO_FLAGS(HAL_WIFI_BLUE_LED_NODE, gpios)

/* WiFi connection parameters */
#define HAL_WIFI_SSID "infrastructure"
#define HAL_WIFI_PSK "WaESjCqZt26A"

#define HAL_WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

extern void hal_wifi_init(void);

#endif