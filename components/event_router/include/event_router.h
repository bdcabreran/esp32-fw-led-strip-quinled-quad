#ifndef EVENT_ROUTER_H
#define EVENT_ROUTER_H

#include "esp_err.h"
#include "stdint.h"


#define EVENT_MAX_PAYLOAD_LEN  (32*2)

// Define macro for button events
#define BUTTON_EVENTS \
    EVT_BUTTON_SINGLE_PRESS, \
    EVT_BUTTON_LONG_PRESS, \
    EVT_BUTTON_RELEASE

#define BUTTON_EVENTS_NAME \
    "EVT_BUTTON_SINGLE_PRESS", \
    "EVT_BUTTON_LONG_PRESS", \
    "EVT_BUTTON_RELEASE"

// Define macro for touch events
#define TOUCH_SENSOR_EVENTS \
    EVT_TOUCH_SENSOR_SINGLE_PRESS, \
    EVT_TOUCH_SENSOR_LONG_PRESS, \
    EVT_TOUCH_SENSOR_RELEASE

#define TOUCH_SENSOR_EVENTS_NAME \
    "EVT_TOUCH_SENSOR_SINGLE_PRESS", \
    "EVT_TOUCH_SENSOR_LONG_PRESS", \
    "EVT_TOUCH_SENSOR_RELEASE"



typedef enum
{
    TASK_INVALID,
    TASK_BUTTON,
    TASK_TOUCH_SENSOR,
    TASK_LED_CONTROL,
    TASK_LAST, 
}task_id_t;

typedef enum
{
   EVENT_INVALID, 
   BUTTON_EVENTS,
   TOUCH_SENSOR_EVENTS,
   EVENT_LAST
}event_id_t;

typedef struct 
{
    uint32_t button_number;
}button_payload_t;

typedef struct
{
    uint16_t len;
    union
    {
        button_payload_t button;
        uint8_t buffer[EVENT_MAX_PAYLOAD_LEN];
    };
}event_payload_t;

typedef struct
{
    task_id_t  src;
    task_id_t  dest;
    event_id_t id;
    event_payload_t payload;
}event_t;


#define IS_VALID_EVENT_ID(evt) (evt > EVENT_INVALID && evt < EVENT_LAST )
#define IS_VALID_TASK_ID(id) (id > TASK_INVALID && id < TASK_LAST)
#define EVENT_ITEM_SIZE_BYTES         sizeof(event_t)


void event_router_init(void);
esp_err_t event_router_write(const event_t *event);
char *event_router_get_id_name(event_id_t id);
char *event_router_get_task_name(task_id_t id);


#endif