#include "led_control.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_router.h"


static QueueHandle_t led_control_evt_queue;

#define LED_CTRL_DEBUG_ENABLE
#ifndef LED_CTRL_DEBUG_ENABLE
    #define LED_CTRL_LOGI(...) do {} while(0)
    #define LED_CTRL_LOGE(...) do {} while(0)
    #define LED_CTRL_LOGW(...) do {} while(0)
if
#else
// Tag used for ESP serial console messages
	static const char *TAG = "[LED_CONTROL]";
	#define LED_CTRL_LOGI(...) ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_PURPLE) __VA_ARGS__)
	#define LED_CTRL_LOGE(...) ESP_LOGE(TAG, LOG_COLOR(LOG_COLOR_PURPLE) __VA_ARGS__)
	#define LED_CTRL_LOGW(...) ESP_LOGW(TAG, LOG_COLOR(LOG_COLOR_PURPLE) __VA_ARGS__)
#endif


// static void button_notify_event(uint32_t gpio_num, uint32_t btn_event)
// {
//     event_t event = {0}; 
//     event.src = TASK_BUTTON;
//     event.dest = TASK_LED_CONTROL;
//     event.id = EVENT_INVALID;
//     event.payload.len = 0;

//     switch (gpio_num)
//     {
//         case BUTTON1_GPIO: event.id = btn_event; break;
//     default:
//         break;
//     }

//     if(event.id != EVENT_INVALID)
//         event_router_write(&event);
// }



/**
 * @brief Task to handle button events
 * 
 * @param arg 
 */
void led_control_task(void* arg)
{

    for (;;) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Initialize button driver
 * 
 */
void led_control_init(void)
{
    led_control_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 2, NULL);
    LED_CTRL_LOGI("Initialized");
}

