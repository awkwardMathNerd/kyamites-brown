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

#include <methane_sensor_srv_handler.h>
#include <bluetooth/mesh/boards/argon_prov.h>

/* Macros ----------------------------------------------------- */

/* Variables -------------------------------------------------- */

/* Prototypes ------------------------------------------------ */

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("failed ready BT (err %d)", err);
		return;
	}

	LOG_INF("BT ok");

	err = bt_mesh_init(bt_mesh_argon_prov_init(), methane_sensor_srv_handler());
	if (err) {
		LOG_ERR("failed provision methane sensor server on argon (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	LOG_INF("BT Mesh ok");
}

void main(void) {

	int err;

	const struct device *usb = device_get_binding("CDC_ACM_0");
    if (!usb) {
		LOG_ERR("failed get CDC ACM device");
        return;
    }

    err = usb_enable(NULL);
    if (err) {
        LOG_ERR("failed enable usb (err %d)", err);
        return;
    } 

	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("failed enable BT (err %d)", err);
		return;
	}

	LOG_INF("Init ok");
}
