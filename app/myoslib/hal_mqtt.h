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
* INTERNAL FUNCTIONS
************************************************************
* mqtt_evt_handler() - Event handler for MQTT actions
* get_mqtt_payload() - Return the desired payload
* get_mqtt_topic() - Return the desired topic
* publish() - MQTT publish function call
* broker_init() - Initialise the MQTT broker
* client_init() - Initialise the MQTT client
* prepare_fds() - Set all fds
* clear_fds() - Set fd flag to 0
* wait() - MQTT waiting function to poll the socket
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
#include <random/rand32.h>

#include "meas_packet.h"

// #define SERVER_ADDR		        "99.83.172.119"
#define SERVER_ADDR				"75.2.83.130"
#define SERVER_PORT		        1883
#define APP_MQTT_BUFFER_SIZE	256
#define MQTT_CLIENTID		    "YEET"

#define MQTT_USERNAME "Token"
#define MQTT_PASSWORD "3ec354ac-4a20-408a-b076-6e29dbb3334c"

void mqtt_evt_handler(struct mqtt_client *const client, const struct mqtt_evt *evt);

static char *get_mqtt_payload(enum mqtt_qos qos);

static char *get_mqtt_topic(void);

int publish(struct mqtt_client *client, enum mqtt_qos qos);

void broker_init(void);

void client_init(struct mqtt_client *client);

void prepare_fds(struct mqtt_client *client);

void clear_fds(void);

int wait(int timeout);

extern void hal_mqtt_init(void);

extern void hal_mqtt_publish(void);

#endif