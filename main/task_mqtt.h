#ifndef __TASK_MQTT_H__
#define __TASK_MQTT_H__

#include <stdint.h>

void mqtt_task_start(void);
int mqtt_task_pusblish(const char *topic, const char *data, uint16_t len);

#endif
