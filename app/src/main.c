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

#include "hal_wifi.h"
#include "hal_mqtt.h"

/**
 * The entry point for the main thread of the program
 */
void main(void)
{

	hal_wifi_init();
	hal_mqtt_init();

	uint32_t c = 400;

	/* Enter infinite loop */
	while (1) {

		/* Give resources to the other threads */
		hal_mqtt_payload_update(1.25f, c);
		c += 100;
		hal_mqtt_publish();
		k_msleep(5000);
	}
}
