/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <zephyr.h>
#include <net/socket.h>
#include <net/mqtt.h>
#include <string.h>
#include <errno.h>

#define ZEPHYR_ADDR		        "192.168.1.101"
#define SERVER_ADDR		        "192.168.1.10"
#define SERVER_PORT		        1883
#define APP_MQTT_BUFFER_SIZE	128
#define MQTT_CLIENTID		    "kyamites_brown_publisher"

extern int publish(struct mqtt_client *client, enum mqtt_qos qos);

extern void broker_init(void);

extern void client_init(struct mqtt_client *client);


#endif
