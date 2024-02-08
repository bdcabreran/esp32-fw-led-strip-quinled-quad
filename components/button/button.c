#include "button.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_router.h"
#include "esp_timer.h"

// if marked as 0 then long press will be detected on timer expiry
#define DETECT_LONG_PRESS_ON_BUTTON_RELEASE 0

#define BUTTON1_GPIO            (12) // GPIO number for button 1 (QuinLED-Dig-Uno Q2 - GPIO12)
#define DEBOUNCE_TIME_MS        (50) // Debounce time in milliseconds
#define LONG_PRESS_THRESHOLD_MS (300) // Long press threshold in milliseconds

static QueueHandle_t button_evt_queue;

#define BUTTON_DEBUG_ENABLE
#ifndef BUTTON_DEBUG_ENABLE
    #define BUTTON_LOGI(...) do {} while(0)
    #define BUTTON_LOGE(...) do {} while(0)
    #define BUTTON_LOGW(...) do {} while(0)
if
#else
// Tag used for ESP serial console messages
	static const char *TAG = "[BUTTON]";
	#define BUTTON_LOGI(...) ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_BLUE) __VA_ARGS__)
	#define BUTTON_LOGE(...) ESP_LOGE(TAG, LOG_COLOR(LOG_COLOR_BLUE) __VA_ARGS__)
	#define BUTTON_LOGW(...) ESP_LOGW(TAG, LOG_COLOR(LOG_COLOR_BLUE) __VA_ARGS__)
#endif

static void button_notify_event(uint32_t gpio_num, uint32_t btn_event)
{
    event_t event = {0}; 
    event.src = TASK_BUTTON;
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


void button_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
}


/**
 * @brief Initialize button driver
 * 
 */
static void button_driver_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io_conf.pin_bit_mask = (1ULL << BUTTON1_GPIO);
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON1_GPIO, button_isr_handler, (void *)BUTTON1_GPIO);
}


#if DETECT_LONG_PRESS_ON_BUTTON_RELEASE
/**
 * @brief Task to handle button events
 * 
 * @param arg 
 */
void button_press_handler_task(void* arg)
{
    esp_log_level_set("gpio", ESP_LOG_NONE);
    button_driver_init();

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
                            BUTTON_LOGI("Button %d evt [%s] time = [%d]", gpio_num, "EVT_BUTTON_LONG_PRESS", duration);
                            button_notify_event(gpio_num, EVT_BUTTON_LONG_PRESS);
                        } else {
                            // Single press event
                            BUTTON_LOGI("Button %d evt [%s] time = [%d]", gpio_num, "EVT_BUTTON_SINGLE_PRESS", duration);
                            button_notify_event(gpio_num, EVT_BUTTON_SINGLE_PRESS);
                        }
                        // Button release event
                        BUTTON_LOGI("Button %d evt [%s]", gpio_num, "EVT_BUTTON_RELEASE");
                        button_notify_event(gpio_num, EVT_BUTTON_RELEASE);
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
        .arg = BUTTON1_GPIO, // Can be used to pass data to the callback function
        .name = "long_press_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &long_press_timer));
}

// Timer callback function
static void long_press_timer_callback(void* arg) {
    uint32_t gpio_num = (uint32_t)arg; // Cast and retrieve the GPIO number if needed
    // Long press action
    BUTTON_LOGI("Button %d evt [%s] via timer", gpio_num, "EVT_BUTTON_LONG_PRESS");
    button_notify_event(gpio_num, EVT_BUTTON_LONG_PRESS);
    long_press_handled = true;
}

// detect long press on timer expiry
void button_press_handler_task(void* arg) {
    esp_log_level_set("gpio", ESP_LOG_NONE);
    button_driver_init();
    long_press_timer_init(); // Initialize the long press timer

    uint32_t gpio_num;
    uint32_t last_state = 1;
    uint32_t debounce_time = (DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);

    for (;;) {
        if (xQueueReceive(button_evt_queue, &gpio_num, portMAX_DELAY)) {
            uint32_t curr_state = gpio_get_level(gpio_num);
            if (curr_state != last_state) {
                vTaskDelay(debounce_time);
                curr_state = gpio_get_level(gpio_num);
                if (curr_state != last_state) {
                    if (curr_state == 0) {  // Button is pressed
                        // Start the long press timer
                        long_press_handled = false; // Reset the flag
                        esp_timer_start_once(long_press_timer, LONG_PRESS_THRESHOLD_MS * 1000);
                    } else { // Button is released
                        // Stop the long press timer if it's running
                        esp_timer_stop(long_press_timer);

                        if(long_press_handled == false)
                        {
                            // Trigger the single press event immediately, assuming timer did not expire
                            BUTTON_LOGI("Button %d evt [%s]", gpio_num, "EVT_BUTTON_SINGLE_PRESS");
                            button_notify_event(gpio_num, EVT_BUTTON_SINGLE_PRESS);
                        }

                        // Button release event
                        BUTTON_LOGI("Button %d evt [%s]", gpio_num, "EVT_BUTTON_RELEASE");
                        button_notify_event(gpio_num, EVT_BUTTON_RELEASE);
                    }
                    last_state = curr_state;
                }
            }
        }
    }
}
#endif


/**
 * @brief Initialize button driver
 * 
 */
void button_init(void)
{
    button_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(button_press_handler_task, "button_press_handler_task", 2048, NULL, 2, NULL);
    BUTTON_LOGI("Initialized");
}

