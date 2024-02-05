#include "button.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "event_router.h"

#define TOUCH_SENSOR_GPIO        (26) // GPIO number for TTP223B touch sensor
#define DEBOUNCE_TIME_MS         (50)  // Debounce time in milliseconds, adjust if necessary
#define LONG_PRESS_THRESHOLD_MS  (300) // Long press threshold in milliseconds


static QueueHandle_t touch_evt_queue;

// Logging macros remain unchanged
#define TOUCH_SENSOR_DEBUG_ENABLE
#ifdef TOUCH_SENSOR_DEBUG_ENABLE
    static const char *TAG = "[TOUCH_SENSOR]";
    #define TOUCH_LOGI(...) ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_BROWN) __VA_ARGS__)
    #define TOUCH_LOGE(...) ESP_LOGE(TAG, LOG_COLOR(LOG_COLOR_BROWN) __VA_ARGS__)
    #define TOUCH_LOGW(...) ESP_LOGW(TAG, LOG_COLOR(LOG_COLOR_BROWN) __VA_ARGS__)
#else
    #define TOUCH_LOGI(...) do {} while(0)
    #define TOUCH_LOGE(...) do {} while(0)
    #define TOUCH_LOGW(...) do {} while(0)
#endif


void touch_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(touch_evt_queue, &gpio_num, NULL);
}

static void touch_notify_event(uint32_t gpio_num, uint32_t touch_event)
{
    event_t event = {0}; 
    event.src = TASK_TOUCH_SENSOR;
    event.dest = TASK_LED_CONTROL;
    event.id = EVENT_INVALID;
    event.payload.len = 0;

    switch (gpio_num)
    {
        case BUTTON1_GPIO: event.id = btn_event; break;
    default:
        break;
    }

    if(event.id != EVENT_INVALID)
        event_router_write(&event);
}

static void touch_sensor_driver_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Trigger on both rising and falling edges
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // TTP223B module typically includes pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << TOUCH_SENSOR_GPIO);
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(TOUCH_SENSOR_GPIO, touch_isr_handler, (void*)TOUCH_SENSOR_GPIO);
}

/**
 * @brief Task to handle button events
 * 
 * @param arg 
 */
void touch_task(void* arg)
{
    esp_log_level_set("gpio", ESP_LOG_NONE);
    touch_sensor_driver_init();

    uint32_t gpio_num;
    uint32_t last_state = 1;
    uint32_t debounce_time = (DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);
    uint32_t press_time = 0;
    uint32_t long_press_threshold = (LONG_PRESS_THRESHOLD_MS / portTICK_PERIOD_MS);

    for (;;) {
        if (xQueueReceive(button_evt_queue, &gpio_num, portMAX_DELAY)) {
            uint32_t curr_state = gpio_get_level(gpio_num);
            if (curr_state != last_state) {
                vTaskDelay(debounce_time);
                curr_state = gpio_get_level(gpio_num);
                if (curr_state != last_state) {

                    if (curr_state == 0) {  // Button is pressed
                        press_time = xTaskGetTickCount();  // Record the press time
                    }
                    else // Button is released
                    {
                        uint32_t release_time = xTaskGetTickCount();
                        uint32_t duration = release_time - press_time;
                        if (duration > long_press_threshold) {
                            // Long press event
                            touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_LONG_PRESS);
                            TOUCH_LOGI("Touch Sensor %d evt [%s] time = [%d]", gpio_num, "EVT_TOUCH_SENSOR_LONG_PRESS", duration);
                        } else {
                            // Single press event
                            touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_SINGLE_PRESS);
                            TOUCH_LOGI("Touch Sensor %d evt [%s] time = [%d]", gpio_num, "EVT_TOUCH_SENSOR_SINGLE_PRESS", duration);
                        }
                        // Button release event
                        touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_RELEASE);
                        TOUCH_LOGI("Touch Sensor %d evt [%s]", gpio_num, "EVT_TOUCH_SENSOR_RELEASE");
                    }

                    last_state = curr_state;
                }
            }
        }
    }
}

void touch_init(void)
{
    touch_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(touch_task, "touch_task", 2048, NULL, 10, NULL);
    TOUCH_LOGI("Initialized");
}

