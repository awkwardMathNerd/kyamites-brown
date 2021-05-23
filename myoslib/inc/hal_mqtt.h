/**
************************************************************
* @file     myoslib/hal_mqtt.h   
* @author   Brendon Duncan 44826482
* @date     23/05/2021
* @brief    Header file for MQTT functionality
* REFERNCE:    
************************************************************
* EXTERNAL FUNCTIONS
************************************************************
* hal_mqtt_init() - Initialse the MQTT connection to Tago
* hal_mqtt_publish() - MQTT publishing function 
************************************************************
*/

#ifndef HAL_MQTT_H
#define HAL_MQTT_H

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <net/socket.h>
#include <net/mqtt.h>
#include <stdio.h>
#include <random/rand32.h>

/* NanoPB stuff */
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "meas_packet.pb.h"

#define HAL_MQTT_SERVER_ADDR				"75.2.83.130"
#define HAL_MQTT_SERVER_PORT		        1883
#define HAL_MQTT_APP_MQTT_BUFFER_SIZE	    256
#define HAL_MQTT_CLIENTID		        "YEET"

#define HAL_MQTT_USERNAME "Token"
#define HAL_MQTT_PASSWORD "3ec354ac-4a20-408a-b076-6e29dbb3334c"

extern void hal_mqtt_init(void);
extern void hal_mqtt_publish(void);
extern void hal_mqtt_payload_update(float vref, uint32_t concentration);

#endif