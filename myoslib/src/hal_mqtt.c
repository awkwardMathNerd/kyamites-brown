/**
************************************************************
* @file     myoslib/hal_mqtt.c   
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

#include "hal_mqtt.h"

LOG_MODULE_REGISTER(hal_mqtt);

/* MQTT rx an tx buffers */
static uint8_t hal_mqtt_rx_buffer[HAL_MQTT_APP_MQTT_BUFFER_SIZE];
static uint8_t hal_mqtt_tx_buffer[HAL_MQTT_APP_MQTT_BUFFER_SIZE];

/* MQTT payload variable */
static char hal_mqtt_payload[100];

/* MQTT username and passwords */
struct mqtt_utf8 hal_mqtt_password;
struct mqtt_utf8 hal_mqtt_username;

/* The MQTT client struct */
static struct mqtt_client hal_mqtt_client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage hal_mqtt_broker;
static bool hal_mqtt_connected = false;

/* MQTT FDs */
struct zsock_pollfd hal_mqtt_fds[1];
int hal_mqtt_nfds;

/**
 * @brief Prepard fds
 *
 * Sets all file descriptors for operations
 *
 * @param client Pointer to the MQTT client struct
 *
 * @retval None.
 */
void hal_mqtt_prepare_fds(struct mqtt_client *client) {

	if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		hal_mqtt_fds[0].fd = client->transport.tcp.sock;
	}

	hal_mqtt_fds[0].events = ZSOCK_POLLIN;
	hal_mqtt_nfds = 1;
}

/**
 * @brief Clear all fds
 *
 * All Fds are set to 0
 *
 * @param None
 *
 * @retval None.
 */
void hal_mqtt_clear_fds(void) {

	hal_mqtt_nfds = 0;
}

/**
 * @brief Callback handler for MQTT
 *
 * Checks the state of the MQTT event after operations
 *
 * @param client Pointer to the MQTT client struct
 * @param evt Pointer to the MQTT event struct 
 *
 * @retval None.
 */
void hal_mqtt_mqtt_evt_handler(struct mqtt_client *const client,
		      const struct mqtt_evt *evt) {

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
		}

		hal_mqtt_connected = true;
		LOG_INF("MQTT client connected!");

		break;

	case MQTT_EVT_DISCONNECT:
		hal_mqtt_connected = false;
		hal_mqtt_clear_fds();

		break;

	case MQTT_EVT_PUBREC:
		if (evt->result != 0) {
			break;
		}

		const struct mqtt_pubrel_param rel_param = {
			.message_id = evt->param.pubrec.message_id
		};

		mqtt_publish_qos2_release(client, &rel_param);
		break;

	default:
		break;
	}
}

/**
 * @brief Funtion to return payload string
 *
 * Returns char* for the desired MQTT payload
 *
 * @param qos MQTT quality of service enum
 *
 * @retval char* pyload string
 */
static char *hal_mqtt_get_mqtt_payload(enum mqtt_qos qos) {

	return hal_mqtt_payload;
}

/**
 * @brief Function to return topic string
 *
 * Returns char* for the desired MQTT payload
 *
 * @param None
 *
 * @retval char* topic string
 */
static char *hal_mqtt_get_mqtt_topic(void) {

	return "methane";
}

/**
 * @brief Publish the data to the MQTT destination
 *
 * Collects the topic, payload and qos to construct a param struct and then
 * publish to the client given the provided parameters.
 *
 * @param client Pointer to the MQTT client struct
 * @param qos MQTT quality of service enum 
 *
 * @retval The success integer from mqtt_publish()
 */
int hal_mqtt_publish_internal(struct mqtt_client *client, enum mqtt_qos qos) {
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (uint8_t *)hal_mqtt_get_mqtt_topic();
	param.message.topic.topic.size = strlen(param.message.topic.topic.utf8);
	param.message.payload.data = hal_mqtt_get_mqtt_payload(qos);
	param.message.payload.len = strlen(param.message.payload.data);
	param.message_id = sys_rand32_get();
	param.message_id = 1;
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}

/**
 * @brief Initialise the broker
 *
 * Create and initialise the broker for operation
 *
 * @param None
 *
 * @retval None.
 */
void hal_mqtt_broker_init(void) {

	struct sockaddr_in *broker4 = (struct sockaddr_in *)&hal_mqtt_broker;

	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(HAL_MQTT_SERVER_PORT);
	zsock_inet_pton(AF_INET, HAL_MQTT_SERVER_ADDR, &broker4->sin_addr);
}

/**
 * @brief Initialise the client
 *
 * Initialies the client and broker. Connects all required handlers, buffers
 * and username and password data to enable MQTT operation.
 *
 * @param client Pointer to the MQTT client struct
 *
 * @retval None.
 */
void hal_mqtt_client_init(struct mqtt_client *client) {

	mqtt_client_init(client);

	hal_mqtt_broker_init();

	hal_mqtt_password.utf8 = (uint8_t *)HAL_MQTT_PASSWORD;
	hal_mqtt_password.size = strlen(HAL_MQTT_PASSWORD);

	hal_mqtt_username.utf8 = (uint8_t *)HAL_MQTT_USERNAME;
	hal_mqtt_username.size = strlen(HAL_MQTT_USERNAME);


	/* MQTT client configuration */
	client->broker = &hal_mqtt_broker;
	client->evt_cb = hal_mqtt_mqtt_evt_handler;
	client->client_id.utf8 = (uint8_t *)HAL_MQTT_CLIENTID;
	client->client_id.size = strlen(HAL_MQTT_CLIENTID);
	client->password = &hal_mqtt_password;
	client->user_name = &hal_mqtt_username;

	/* MQTT buffers configuration */
	client->rx_buf = hal_mqtt_rx_buffer;
	client->rx_buf_size = sizeof(hal_mqtt_rx_buffer);
	client->tx_buf = hal_mqtt_tx_buffer;
	client->tx_buf_size = sizeof(hal_mqtt_tx_buffer);

	/* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

/**
 * @brief Callback handler for MQTT
 *
 * Wait function to poll the TCP socket
 *
 * @param timeout Timeout as an integer
 *
 * @retval Return status from zsock_poll
 */
int hal_mqtt_wait(int timeout) {

	int ret = 0;

	if (hal_mqtt_nfds > 0) {
		ret = zsock_poll(hal_mqtt_fds, hal_mqtt_nfds, timeout);
		if (ret < 0) {
			LOG_ERR("poll error: %d", errno);
		}
	}

	return ret;
}

/**
 * @brief Initialise the MQTT connection
 *
 * Connects to client and prepares fds, then waits for mqtt_input from the 
 * client
 *
 * @param meas_packet_message Data packet including the calibration
                                value and concentration
 *
 * @retval None.
 */
void hal_mqtt_payload_update(float vref, uint32_t concentration) {

    uint8_t buff[100];
    meas_packet_message message = meas_packet_message_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buff, sizeof(buff));

    message.methane_concentration =  concentration;
    message.calibration_value = vref;

    bool status = pb_encode(&stream, meas_packet_message_fields, &message);

    if (status) {

        for (int i = 0; i < stream.bytes_written; i++) {

            sprintf(hal_mqtt_payload + (2 * i), "%02x", buff[i]);
        }
        hal_mqtt_payload[2*stream.bytes_written] = 0;
    }
}

/**
 * @brief Initialise the MQTT connection
 *
 * Connects to client and prepares fds, then waits for mqtt_input from the 
 * client
 *
 * @param None
 *
 * @retval None.
 */
void hal_mqtt_init(void) {

    hal_mqtt_client_init(&hal_mqtt_client_ctx);
	LOG_INF("CONNECT: %d", mqtt_connect(&hal_mqtt_client_ctx));

	hal_mqtt_prepare_fds(&hal_mqtt_client_ctx);

	if (hal_mqtt_wait(2000)) {
		mqtt_input(&hal_mqtt_client_ctx);
	}
}

/**
 * @brief Publish the MQTT data
 *
 * Publishes the desired MQTT data to the client
 *
 * @param None
 * @retval None.
 */
void hal_mqtt_publish(void) {

    LOG_INF("Publish: %d", hal_mqtt_publish_internal(&hal_mqtt_client_ctx, 1));
}