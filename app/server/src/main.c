/**
***************************************************************
* @file apps/p3/MOBILE/src/main.c
* @author Lachlan Smith - s4482220
* @date 08032021
* @brief Practical 3
*************************************************************** */

/* Includes --------------------------------------------------- */

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>

/* Typedefs --------------------------------------------------- */

/* Defines ---------------------------------------------------- */

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <usb/usb_device.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh.h>

/* Macros ----------------------------------------------------- */

/* Variables -------------------------------------------------- */

// static const struct bt_mesh_sensor_column columns[] = {
// 	{ { 0 }, { 20 } },
// 	{ { 20 }, { 25 } },
// 	{ { 25 }, { 30 } },
// 	{ { 30 }, { 100 } },
// };

static const struct device *crickit;
// static uint32_t tot_temp_samps;
// static uint32_t col_samps[ARRAY_SIZE(columns)];

static int32_t prev_pres;

/* Prototypes ------------------------------------------------ */

static int crickit_methane_sensor_voltage_get(struct bt_mesh_sensor *sensor,
			 struct bt_mesh_msg_ctx *ctx, struct sensor_value *rsp) {
				 
	// int err;

	// sensor_sample_fetch(dev);

	// err = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, rsp);

	// if (err) {
	// 	printk("Error getting temperature sensor data (%d)\n", err);
	// 	return err;
	// }

	// for (int i = 0; i < ARRAY_SIZE(columns); ++i) {
	// 	if (bt_mesh_sensor_value_in_column(rsp, &columns[i])) {
	// 		col_samps[i]++;
	// 		break;
	// 	}
	// }

	// tot_temp_samps++;

	return 0;
}

static struct bt_mesh_sensor crickit_methane_sensor_voltage = {
	.type = &bt_mesh_sensor_avg_output_voltage,
	.get = crickit_methane_sensor_voltage_get,
};

static struct bt_mesh_sensor *const sensors[] = {
	&crickit_methane_sensor_voltage,
};

static struct bt_mesh_sensor_srv sensor_srv =
	BT_MESH_SENSOR_SRV_INIT(sensors, ARRAY_SIZE(sensors));

static bool attention;

static void attention_on(struct bt_mesh_model *mod)
{
	attention = true;
}

static void attention_off(struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(1,
		     BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV,
					BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
					BT_MESH_MODEL_SENSOR_SRV(&sensor_srv)),
		     BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void) {

	crickit = device_get_binding(DT_LABEL(DT_INST(0, adafruit_crickit)));
	if (!crickit) {
		printk("failed get CRICKIT device\r\n");
	}

	return &comp;
}

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

void main(void) {

	int err;

	const struct device *usb = device_get_binding("CDC_ACM_0");
    if (!usb) {
		printk("failed get USB device\r\n");
        return;
    }

    err = usb_enable(NULL);
    if (err) {
        printk("failed initialise usb cdc acm\r\n");

        return;
    } 

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}
