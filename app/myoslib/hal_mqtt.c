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

#include "hal_mqtt.h"

LOG_MODULE_REGISTER(app);

/* MQTT rx an tx buffers */
static uint8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[APP_MQTT_BUFFER_SIZE];

/* MQTT payload variable */
static char payload[100];

/* MQTT username and passwords */
struct mqtt_utf8 password;
struct mqtt_utf8 username;

/* The MQTT client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;
static bool connected = false;

/* MQTT FDs */
struct zsock_pollfd fds[1];
int nfds;

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
void mqtt_evt_handler(struct mqtt_client *const client,
		      const struct mqtt_evt *evt) {

	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
		}

		connected = true;
		LOG_INF("MQTT client connected!");

		break;

	case MQTT_EVT_DISCONNECT:
		// LOG_INF("MQTT client disconnected %d", evt->result);

		connected = false;
		clear_fds();

		break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			// LOG_ERR("MQTT PUBACK error %d", evt->result);
			break;
		}

		// LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

		break;

	case MQTT_EVT_PUBREC:
		if (evt->result != 0) {
			// LOG_ERR("MQTT PUBREC error %d", evt->result);
			break;
		}

		// LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

		const struct mqtt_pubrel_param rel_param = {
			.message_id = evt->param.pubrec.message_id
		};

		err = mqtt_publish_qos2_release(client, &rel_param);
		if (err != 0) {
			// LOG_ERR("Failed to send MQTT PUBREL: %d", err);
		}

		break;

	case MQTT_EVT_PUBCOMP:
		if (evt->result != 0) {
			// LOG_ERR("MQTT PUBCOMP error %d", evt->result);
			break;
		}

		// LOG_INF("PUBCOMP packet id: %u",
			// evt->param.pubcomp.message_id);

		break;

	case MQTT_EVT_PINGRESP:
		// LOG_INF("PINGRESP packet");
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
static char *get_mqtt_payload(enum mqtt_qos qos) {

	return payload;
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
static char *get_mqtt_topic(void) {

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
int publish(struct mqtt_client *client, enum mqtt_qos qos) {
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (uint8_t *)get_mqtt_topic();
	param.message.topic.topic.size = strlen(param.message.topic.topic.utf8);
	param.message.payload.data = get_mqtt_payload(qos);
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
void broker_init(void) {

	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(SERVER_PORT);
	zsock_inet_pton(AF_INET, SERVER_ADDR, &broker4->sin_addr);
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
void client_init(struct mqtt_client *client) {

	mqtt_client_init(client);

	broker_init();

	password.utf8 = (uint8_t *)MQTT_PASSWORD;
	password.size = strlen(MQTT_PASSWORD);

	username.utf8 = (uint8_t *)MQTT_USERNAME;
	username.size = strlen(MQTT_USERNAME);


	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;
	client->client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
	client->client_id.size = strlen(MQTT_CLIENTID);
	client->password = &password;
	client->user_name = &username;
	//client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

/**
 * @brief Prepard fds
 *
 * Sets all file descriptors for operations
 *
 * @param client Pointer to the MQTT client struct
 *
 * @retval None.
 */
void prepare_fds(struct mqtt_client *client) {

	if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds[0].fd = client->transport.tcp.sock;
	}

	fds[0].events = ZSOCK_POLLIN;
	nfds = 1;
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
void clear_fds(void) {

	nfds = 0;
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
int wait(int timeout) {

	int ret = 0;

	if (nfds > 0) {
		ret = zsock_poll(fds, nfds, timeout);
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
void hal_payload_update(float vref, uint32_t concentration) {

    uint8_t buff[100];
    meas_packet_message message = meas_packet_message_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buff, sizeof(buff));

    message.methane_concentration =  concentration;
    message.calibration_value = vref;

    bool status = pb_encode(&stream, meas_packet_message_fields, &message);

    if (status) {

        for (int i = 0; i < stream.bytes_written; i++) {

            sprintf(payload + (2 * i), "%02x", buff[i]);
        }
        payload[2*stream.bytes_written] = 0;
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

    client_init(&client_ctx);
	LOG_INF("CONNECT: %d", mqtt_connect(&client_ctx));

	prepare_fds(&client_ctx);

	if (wait(2000)) {
		mqtt_input(&client_ctx);
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

    LOG_INF("Publish: %d", publish(&client_ctx, 1));
}