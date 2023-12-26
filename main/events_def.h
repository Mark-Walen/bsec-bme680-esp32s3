#ifndef __EVENT_HANDLER__H__
#define __EVENT_HANDLER__H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* FreeRTOS event group to signal when we are connected*/
extern EventGroupHandle_t s_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_EVT_CONNECTED_BIT      BIT0
#define WIFI_EVT_FAIL_BIT           BIT1
#define MQTT_EVT_ERROR              BIT2

#endif
