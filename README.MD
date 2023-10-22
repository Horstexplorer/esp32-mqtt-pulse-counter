# MQTT pulse counter for reading gas meters

## Required Parts
- ESP32
- Reed sensor

## Configuration
The following values need to be adjusted in main.cpp

`IO_SENSOR_PIN` GPIO which the reed sensor connects to  
`IO_RESET_PREFERENCES_PIN` GPIO used to reset internally stored counter values  
`PREFERENCES_TOTAL_PULSES_COUNT_OVERRIDE` Amount of counted pulses to be used as initial value. Default 0  
`PREFERENCES_TOTAL_VOLUME_OVERRIDE` Amount of gas volume to be used as initial value. Default 0  
`PREFERENCES_VOLUME_PER_IMPULSE` Amount of gas volume per pulse in m³  
`WIFI_SSID` WiFi SSID  
`WIFI_PASSWORD` WiFi password  
`PUBSUB_SERVER_ADDRESS` IPv4 of the MQTT server  
`PUBSUB_SERVER_PORT` Port of the MQTT server  
`PUBSUB_USERNAME` MQTT username  
`PUBSUB_PASSWORD` MQTT password  
`PUBSUB_CLIENT_ID` MQTT client id  