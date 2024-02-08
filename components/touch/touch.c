#include "touch.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "event_router.h"


// if marked as 0 then long press will be detected on timer expiry
#define DETECT_LONG_PRESS_ON_TOUCH_RELEASE 0

#define TOUCH_SENSOR_GPIO        (12) // GPIO number for TTP223B touch sensor (QuinLED-Dig-Uno Q2 - GPIO12)
#define DEBOUNCE_TIME_MS         (50)  // Debounce time in milliseconds, adjust if necessary
#define LONG_PRESS_THRESHOLD_MS  (300) // Long press threshold in milliseconds


static QueueHandle_t touch_evt_queue;

// Logging macros remain unchanged
#define TOUCH_SENSOR_DEBUG_ENABLE
#ifdef TOUCH_SENSOR_DEBUG_ENABLE
    static const char *TAG = "[TOUCH_SENSOR]";
    #define TOUCH_LOGI(...) ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_BLUE) __VA_ARGS__)
    #define TOUCH_LOGE(...) ESP_LOGE(TAG, LOG_COLOR(LOG_COLOR_BLUE) __VA_ARGS__)
    #define TOUCH_LOGW(...) ESP_LOGW(TAG, LOG_COLOR(LOG_COLOR_BLUE) __VA_ARGS__)
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
        case TOUCH_SENSOR_GPIO: event.id = touch_event; break;
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

#if DETECT_LONG_PRESS_ON_TOUCH_RELEASE
/**
 * @brief Task to handle button events
 * 
 * @param arg 
 */
void touch_press_handler_task(void* arg)
{
    esp_log_level_set("gpio", ESP_LOG_NONE);
    touch_sensor_driver_init();

    uint32_t gpio_num;
    uint32_t last_state = 1;
    uint32_t debounce_time = (DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);
    uint32_t press_time = 0;
    uint32_t long_press_threshold = (LONG_PRESS_THRESHOLD_MS / portTICK_PERIOD_MS);

    for (;;) {
        if (xQueueReceive(touch_evt_queue, &gpio_num, portMAX_DELAY)) {
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
                            TOUCH_LOGI("Touch Sensor %d evt [%s] time = [%d]", gpio_num, "EVT_TOUCH_SENSOR_LONG_PRESS", duration);
                            touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_LONG_PRESS);
                        } else {
                            // Single press event
                            TOUCH_LOGI("Touch Sensor %d evt [%s] time = [%d]", gpio_num, "EVT_TOUCH_SENSOR_SINGLE_PRESS", duration);
                            touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_SINGLE_PRESS);
                        }
                        // Button release event
                        TOUCH_LOGI("Touch Sensor %d evt [%s]", gpio_num, "EVT_TOUCH_SENSOR_RELEASE");
                        touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_RELEASE);
                    }

                    last_state = curr_state;
                }
            }
        }
    }
}
#else

// Timer handle for long press detection
static esp_timer_handle_t long_press_timer;

// Global or static flag to indicate long press event handling
static bool long_press_handled = false;

// Forward declaration of the timer callback function
static void long_press_timer_callback(void* arg);

// Initialize the long press timer
static void long_press_timer_init() {
    const esp_timer_create_args_t timer_args = {
        .callback = &long_press_timer_callback,
        .arg = (void*)TOUCH_SENSOR_GPIO, // Can be used to pass data to the callback function
        .name = "long_press_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &long_press_timer));
}

// Timer callback function
static void long_press_timer_callback(void* arg) {
    uint32_t gpio_num = (uint32_t)arg; // Cast and retrieve the GPIO number if needed
    // Long press action
    TOUCH_LOGI("Touch Sensor %d evt [%s] via timer", gpio_num, "EVT_TOUCH_SENSOR_LONG_PRESS");
    touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_LONG_PRESS);
    long_press_handled = true;
}

// detect long press on timer expiry
void touch_press_handler_task(void* arg) {
    esp_log_level_set("gpio", ESP_LOG_NONE);
    touch_sensor_driver_init();
    long_press_timer_init(); // Initialize the long press timer

    uint32_t gpio_num;
    uint32_t last_state = 1;
    uint32_t debounce_time = (DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);

    for (;;) {
        if (xQueueReceive(touch_evt_queue, &gpio_num, portMAX_DELAY)) {
            uint32_t curr_state = gpio_get_level(gpio_num);
            if (curr_state != last_state) {
                vTaskDelay(debounce_time);
                curr_state = gpio_get_level(gpio_num);
                if (curr_state != last_state) {
                    if (curr_state == 0) {  // Touch is pressed
                        // Start the long press timer
                        long_press_handled = false; // Reset the flag
                        esp_timer_start_once(long_press_timer, LONG_PRESS_THRESHOLD_MS * 1000);
                    } else { // Touch is released
                        // Stop the long press timer if it's running
                        esp_timer_stop(long_press_timer);

                        if(long_press_handled == false)
                        {
                            // Trigger the single press event immediately, assuming timer did not expire
                            TOUCH_LOGI("Touch Sensor %d evt [%s]", gpio_num, "EVT_BUTTON_SINGLE_PRESS");
                            touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_SINGLE_PRESS);
                        }

                        // Button release event
                        TOUCH_LOGI("Touch Sensor %d evt [%s]", gpio_num, "EVT_TOUCH_SENSOR_RELEASE");
                        touch_notify_event(gpio_num, EVT_TOUCH_SENSOR_RELEASE);
                    }
                    last_state = curr_state;
                }
            }
        }
    }
}
#endif 

void touch_init(void)
{
    touch_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(touch_press_handler_task, "touch_press_handler_task", 2048, NULL, 10, NULL);
    TOUCH_LOGI("Initialized");
}

