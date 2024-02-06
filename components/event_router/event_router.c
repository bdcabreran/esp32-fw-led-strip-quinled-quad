
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"
#include "event_router.h"
#include "led_control.h"

#define EVT_ROUTER_DEBUG_ENABLE (1)
static const char *TAG = "[EVT_ROUTER]";

#ifdef EVT_ROUTER_DEBUG_ENABLE
	#define EVT_ROUTER_LOGI(...) ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_CYAN) __VA_ARGS__)
	#define EVT_ROUTER_LOGE(...) ESP_LOGE(TAG, LOG_COLOR(LOG_COLOR_CYAN) __VA_ARGS__)
	#define EVT_ROUTER_LOGW(...) ESP_LOGW(TAG, LOG_COLOR(LOG_COLOR_CYAN) __VA_ARGS__)
#else
    #define EVT_ROUTER_LOGI(...)
    #define EVT_ROUTER_LOGE(...)
    #define EVT_ROUTER_LOGW(...)
#endif

static QueueHandle_t event_queue = NULL;
static TaskHandle_t event_router_handle = NULL;

const char *task_id_name[TASK_LAST] = 
{
    "TASK_INVALID",
    "TASK_BUTTON",
    "TASK_TOUCH_SENSOR",
    "TASK_LED_CONTROL",
};

const char *event_id_name[] = 
{
    "EVT_INVALID",
    BUTTON_EVENTS_NAME,
    TOUCH_SENSOR_EVENTS_NAME,
};

#define MAX_QUEUE_ITEMS           (15)
#define MAX_QUEUE_SEND_TIMEOUT_MS (5)

static void event_router_task(void *args);


char *event_router_get_id_name(event_id_t id)
{
    return (char *)event_id_name[id];
}

char *event_router_get_task_name(task_id_t id)
{
    return (char *)task_id_name[id];
}


void event_router_init(void) {
    event_queue = xQueueCreate(MAX_QUEUE_ITEMS, sizeof(event_t));
    if (event_queue == NULL) {
        EVT_ROUTER_LOGE("Failed to create event queue");
        return;
    }
    xTaskCreate(event_router_task, "event_router", 2048, NULL, 5, &event_router_handle);
    EVT_ROUTER_LOGI("Initialized");
}


esp_err_t event_router_write(const event_t *event) {
    if (!IS_VALID_EVENT_ID(event->id)) {
        EVT_ROUTER_LOGE("Invalid event ID: %d", event->id);
        return ESP_ERR_INVALID_ARG;
    }
    if (xQueueSend(event_queue, event, pdMS_TO_TICKS(MAX_QUEUE_SEND_TIMEOUT_MS)) == pdPASS) {
        return ESP_OK;
    } else {
        EVT_ROUTER_LOGE("Failed to send event: ID [%d]", event->id);
        return ESP_FAIL;
    }
}

static void print_event_info(event_t *event)
{
    EVT_ROUTER_LOGI("\tsrc     = \t[%s]\n\t \
                     \tdest    = \t[%s]\n\t \
                     \tevt id  = \t[%s]\n\t \
                     \tevt len = \t[%d]",
                    task_id_name[event->src], task_id_name[event->dest],
                    event_id_name[event->id], event->payload.len);
}

esp_err_t event_route_read(event_t *event)
{
    if (xQueueReceive(event_queue, event, portMAX_DELAY) == pdPASS)
    {
        if (IS_VALID_TASK_ID(event->src) && IS_VALID_TASK_ID(event->dest))
        {
            print_event_info(event);
            return ESP_OK;
        }
        else
        {
            EVT_ROUTER_LOGE("Destination or Source Invalid -> [%d][%d]", event->src, event->dest);
            return ESP_FAIL;
        }
    }
    return ESP_FAIL;
}

void event_router_task(void *args)
{
    event_t event;
    while (1)
    {
        if (event_route_read(&event) == ESP_OK)
        {
            switch (event.dest)
            {
            case TASK_BUTTON:
                // button_write_event(&event);
                break;
            case TASK_LED_CONTROL:
                led_control_write_event(&event);
                break;
            case TASK_TOUCH_SENSOR:
                // touch_sensor_write_event(&event);
                break;
            default:
                EVT_ROUTER_LOGW("Task id not handled-> [%d]", event.dest);
                break;
            }
            memset(&event, 0, sizeof(event_t));
        }
    }
}