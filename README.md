# kyamites-brown
Brendon Duncan, Isaac Graham, Bryan Herrington, Lauchlan Smith

## TagoIO
Since this device uses NanoPB to serialize data within its MQTT publish a payload parser is required on the dashboard to interpret this data. The following steps will achieve this:
1. In your device goto the tab 'Payload Parser', select 'Run your own parser' and copy the contents of ``tagIO/device_payload_parse.js`` into the snippet editor
2. Save the payload parser
3. Change device token in ``myoslib/inc/hal_mqtt.h`` (HAL_MQTT_PASSWORD) to match your device
4. Add an Action on TagoIO to subscribe the device to the ``methane`` MQTT topic.

## Code build instructions
To build this code and program the device create a clone of this repository, then in the folder above it run the following commands
```
west init -l kyamites-brown  
west update
```
Then to build the code and program the device run the following from the same folder
```
west build -p auto -b particle_argon kyamites-brown/app  
west flash
```
**WARNING**: The flash command only succeeds when the device is powered, to force this press the tactile pushbutton on the power/timing board.
