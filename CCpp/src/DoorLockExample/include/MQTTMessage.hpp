#ifndef MQTT_MESSAGE_HPP
#define MQTT_MESSAGE_HPP

struct MQTTMessage{
	char* payload;
	interval_t physical_ts;
	interval_t logical_ts;
};

#endif