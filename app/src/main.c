/**
************************************************************
* @file        
* @author      
* @date        
* @brief       
* REFERNCE:    
************************************************************
* EXTERNAL FUNCTIONS
************************************************************
* NONE
************************************************************
*/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(app);

// WiFi network includes
#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_event.h>

// MQTT Stuff
#include <net/socket.h>
#include <net/mqtt.h>
#include <random/rand32.h>

//#define SERVER_ADDR		        "99.83.172.119"
#define SERVER_ADDR				"75.2.83.130"
#define SERVER_PORT		        1883
#define APP_MQTT_BUFFER_SIZE	256
#define MQTT_CLIENTID		    "YEET"

#define MQTT_USERNAME "Token"
#define MQTT_PASSWORD "3ec354ac-4a20-408a-b076-6e29dbb3334c"

static uint8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[APP_MQTT_BUFFER_SIZE];

struct mqtt_utf8 password;
struct mqtt_utf8 username;

/* The mqtt client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;
static bool connected = false;

// LEDs for indicating the status of the WiFi
#define GREEN_LED_NODE    DT_ALIAS(led2)
#define BLUE_LED_NODE     DT_ALIAS(led3)

#define GREEN_LED         DT_GPIO_LABEL(GREEN_LED_NODE, gpios)
#define GREEN_LED_PIN     DT_GPIO_PIN(GREEN_LED_NODE, gpios)
#define GREEN_LED_FLAGS   DT_GPIO_FLAGS(GREEN_LED_NODE, gpios)

#define BLUE_LED          DT_GPIO_LABEL(BLUE_LED_NODE, gpios)
#define BLUE_LED_PIN      DT_GPIO_PIN(BLUE_LED_NODE, gpios)
#define BLUE_LED_FLAGS    DT_GPIO_FLAGS(BLUE_LED_NODE, gpios)

const struct device *green_led_dev = NULL;
const struct device *blue_led_dev = NULL;

/* WiFi connection parameters */
#define WIFI_SSID "infrastructure"
#define WIFI_PSK "WaESjCqZt26A"

#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

static struct net_mgmt_event_callback wifi_mgmt_cb;

volatile uint8_t isConnected = 0;

void mqtt_evt_handler(struct mqtt_client *const client,
		      const struct mqtt_evt *evt)
{
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

static char *get_mqtt_payload(enum mqtt_qos qos)
{
	static char payload[] = "083e1004";

	return payload;
}

static char *get_mqtt_topic(void)
{
	return "methane";
}

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

void broker_init(void) {

	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(SERVER_PORT);
	zsock_inet_pton(AF_INET, SERVER_ADDR, &broker4->sin_addr);
}

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

struct zsock_pollfd fds[1];
int nfds;

void prepare_fds(struct mqtt_client *client)
{
	if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds[0].fd = client->transport.tcp.sock;
	}

	fds[0].events = ZSOCK_POLLIN;
	nfds = 1;
}

void clear_fds(void)
{
	nfds = 0;
}

int wait(int timeout)
{
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
 * WiFi event handler
 */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface)
{

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
 * The entry point for the main thread of the program
 */
void main(void)
{

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

	/* Init the WiFi module and callback */
	net_mgmt_init_event_callback(&wifi_mgmt_cb,
				     wifi_mgmt_event_handler,
				     WIFI_MGMT_EVENTS);

	net_mgmt_add_event_callback(&wifi_mgmt_cb);

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

	client_init(&client_ctx);
	LOG_INF("CONNECT: %d", mqtt_connect(&client_ctx));

	prepare_fds(&client_ctx);

	if (wait(2000)) {
		mqtt_input(&client_ctx);
	}

	/* Enter infinite loop */
	while (1) {

		/* Give resources to the other threads */
		LOG_INF("Publish: %d", publish(&client_ctx, 1));
		k_msleep(5000);
	}
}
