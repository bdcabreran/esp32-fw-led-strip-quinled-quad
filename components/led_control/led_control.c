#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_router.h"
#include "led_control.h"
#include "string.h"
#include "driver/rmt.h"
#include "led_strip.h"


//*********************** USER CONFIGURATION ****************************************//
// LED1 Strip configuration
#define LED1_RMT_TX_CHANNEL   RMT_CHANNEL_0
#define LED1_RMT_TX_GPIO      (16)
#define LED1_STRIP_LED_COUNT  (30)
#define LED1_STRIP_ENABLE     (1)   // 1: enable, 0: disable

// LED2 Strip configuration
#define LED2_RMT_TX_CHANNEL   RMT_CHANNEL_1
#define LED2_RMT_TX_GPIO      (3)
#define LED2_STRIP_LED_COUNT  (30)
#define LED2_STRIP_ENABLE     (0)   // 1: enable, 0: disable

// LED3 Strip configuration
#define LED3_RMT_TX_CHANNEL   RMT_CHANNEL_2
#define LED3_RMT_TX_GPIO      (15)
#define LED3_STRIP_LED_COUNT  (30)
#define LED3_STRIP_ENABLE     (0)   // 1: enable, 0: disable

// LED4 Strip configuration
#define LED4_RMT_TX_CHANNEL   RMT_CHANNEL_3
#define LED4_RMT_TX_GPIO      (4)
#define LED4_STRIP_LED_COUNT  (30)
#define LED4_STRIP_ENABLE     (1)   // 1: enable, 0: disable

// LED Type Configuration
#define LED_TYPE              (LED_STRIP_WS2811_HIGH_SPEED)

// LED Animation Configuration
#define LED_ANIMATION_SPEED   (50) // ms
#define LED_DIM_TRANSITION_TIME (500) // time spend to dim up or down  // ms

//*********************************************************************************** //

static QueueHandle_t led_control_evt_queue;

#define LED_CONTROL_DEBUG_ENABLE
#ifndef LED_CONTROL_DEBUG_ENABLE
    #define LED_CONTROL_LOGI(...) do {} while(0)
    #define LED_CONTROL_LOGE(...) do {} while(0)
    #define LED_CONTROL_LOGW(...) do {} while(0)
if
#else
// Tag used for ESP serial console messages
	static const char *TAG = "[LED_CONTROL]";
	#define LED_CONTROL_LOGI(...) ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_BROWN) __VA_ARGS__)
	#define LED_CONTROL_LOGE(...) ESP_LOGE(TAG, LOG_COLOR(LOG_COLOR_BROWN) __VA_ARGS__)
	#define LED_CONTROL_LOGW(...) ESP_LOGW(TAG, LOG_COLOR(LOG_COLOR_BROWN) __VA_ARGS__)
#endif

static const char *state_name[STATE_LAST] = 
{
    "STATE_INVALID",
    "STATE_LED_CONTROL_OFF", 
    "STATE_LED_CONTROL_ON", 
};

//******************** FUNCTION DEFINITION *****************************************//
static void enter_seq_led_strip_off(led_control_fsm_t *fsm);
static void on_state_led_strip_off(led_control_fsm_t *fsm);
static void on_state_led_strip_on(led_control_fsm_t *fsm);
static void enter_seq_led_strip_on(led_control_fsm_t *fsm);
static void exit_action_led_strip_on(led_control_fsm_t *fsm);
static void entry_action_led_strip_on(led_control_fsm_t *fsm);
static void led_control_print_info(led_control_fsm_t *fsm);
static void led_control_execute_animation(led_control_fsm_t *fsm, led_animation_t led_animation);


static void led_control_execute_animation(led_control_fsm_t *fsm, led_animation_t led_animation)
{
    switch (led_animation)
    {
        case LED_ANIMATION_LED_ON:
        {
            for(int i = LED_STRIP_1; i < LED_STRIPn; i++) {
                if (fsm->iface.strips[i].enable)
                    led_strip_set_all(fsm->iface.strips[i].control, 255, 0, 0);
            }
        } break;

        case LED_ANIMATION_LED_OFF:
        {
            for(int i = LED_STRIP_1; i < LED_STRIPn; i++) {
                if (fsm->iface.strips[i].enable)
                    led_strip_clear_all(fsm->iface.strips[i].control, 500);
            }
        } break;

        case LED_ANIMATION_LED_FORWARD_ON:
        {
            // Code for LED FORWARD ON animation
            uint32_t strip_count = LED_STRIPn;              // Total count including NULLs
            uint32_t red = 255, green = 0, blue = 0;        // Color: Red
            uint32_t delay_ms = fsm->iface.animation_speed; // Delay in milliseconds
            led_strip_t *strips[LED_STRIPn] = {NULL};

            for(int i = LED_STRIP_1; i < LED_STRIPn; i++)
                if (fsm->iface.strips[i].enable) 
                    strips[i] = fsm->iface.strips[i].control;
            
            led_strip_forward_on_sync(strips, strip_count, red, green, blue, delay_ms);
        } break;

        case LED_ANIMATION_LED_BACKWARD_OFF:
        {
            // Code for LED BACKWARD OFF animation
            uint32_t strip_count = LED_STRIPn;              // Total count including NULLs
            uint32_t red = 0, green = 0, blue = 0;           // Color: black
            uint32_t delay_ms = fsm->iface.animation_speed;  // Delay in milliseconds
            led_strip_t *strips[LED_STRIPn] = {NULL};

            for(int i = LED_STRIP_1; i < LED_STRIPn; i++)
                if (fsm->iface.strips[i].enable) 
                    strips[i] = fsm->iface.strips[i].control;

            led_strip_backward_off_sync(strips, strip_count, red, green, blue, delay_ms);   
        }
            break;

        default:
            break;
    }
}


static void led_control_set_state(led_control_fsm_t *fsm, led_control_state_t state)
{
    fsm->state.last = fsm->state.curr;
    fsm->state.curr = state;
    memset(&fsm->event, 0, sizeof(event_t)); // Clear the event
    LED_CONTROL_LOGI("transitioning to state -> [%s]", state_name[state]);
}

static void enter_seq_led_strip_off(led_control_fsm_t *fsm)
{
    led_control_set_state(fsm, STATE_LED_CONTROL_OFF);
    LED_CONTROL_LOGI("enter sequence for state -> [%s]", state_name[fsm->state.curr]);
}

static void enter_seq_led_strip_on(led_control_fsm_t *fsm)
{
    led_control_set_state(fsm, STATE_LED_CONTROL_ON);
    LED_CONTROL_LOGI("enter sequence for state -> [%s]", state_name[fsm->state.curr]);
    entry_action_led_strip_on(fsm);
}

static void entry_action_led_strip_on(led_control_fsm_t *fsm)
{
    LED_CONTROL_LOGI("entry action for state -> [%s]", state_name[fsm->state.curr]);
    LED_CONTROL_LOGI("ON Animation Type   : %d", fsm->iface.on_animation);
    LED_CONTROL_LOGI("Executing LED Strip ON sequence");

    // reset brightness level 
    fsm->iface.current_dim_level = fsm->iface.dim_up_level;
    led_control_execute_animation(fsm, fsm->iface.on_animation);
}

static void exit_action_led_strip_on(led_control_fsm_t *fsm)
{
    LED_CONTROL_LOGI("exit action for state -> [%s]", state_name[fsm->state.curr]);
    LED_CONTROL_LOGI("OFF Animation Type   : %d", fsm->iface.off_animation);
    LED_CONTROL_LOGI("Executing LED Strip OFF sequence");

    led_control_execute_animation(fsm, fsm->iface.off_animation);
}


static void on_state_led_strip_off(led_control_fsm_t *fsm)
{
    // Function implementation goes here
    switch (fsm->event.id)
    {
        case EVT_BUTTON_LONG_PRESS:
        case EVT_BUTTON_SINGLE_PRESS:
        case EVT_TOUCH_SENSOR_LONG_PRESS:
        case EVT_TOUCH_SENSOR_SINGLE_PRESS:
        {
            enter_seq_led_strip_on(fsm);
        } break;
         
        default:
            LED_CONTROL_LOGI("event [%s] not handled in state [%s]",
                        event_router_get_id_name(fsm->event.id),
                        state_name[fsm->state.curr]);
        break;
    }
}

static void led_dim_smooth(led_control_fsm_t *fsm, uint8_t current_brightness, uint8_t target_brightness)
{
    uint32_t strip_count = LED_STRIPn;                      // Total count including NULLs
    uint32_t transition_time_ms = LED_DIM_TRANSITION_TIME;  // Delay in milliseconds
    led_strip_t *strips[LED_STRIPn] = {NULL};

    for(int i = LED_STRIP_1; i < LED_STRIPn; i++)
        if (fsm->iface.strips[i].enable) 
            strips[i] = fsm->iface.strips[i].control;

    led_strip_dim_smooth(strips, strip_count, current_brightness, target_brightness, transition_time_ms);
}

static void on_state_led_strip_on(led_control_fsm_t *fsm)
{
    switch (fsm->event.id)
    {
        case EVT_BUTTON_LONG_PRESS:
        case EVT_TOUCH_SENSOR_LONG_PRESS:
        {
            uint8_t current_brightness = (fsm->iface.current_dim_level - 1)*10; // 0-100%
            uint8_t target_brightness = 0;
            
            if (fsm->iface.current_dim_level == fsm->iface.dim_up_level)
            {
                fsm->iface.current_dim_level = fsm->iface.dim_down_level; // dim down
                target_brightness = (fsm->iface.current_dim_level - 1)*10;
                led_dim_smooth(fsm, current_brightness, target_brightness);
            }
            else
            {
                fsm->iface.current_dim_level = fsm->iface.dim_up_level;   // dim up
                target_brightness = (fsm->iface.current_dim_level - 1)*10;
                led_dim_smooth(fsm, current_brightness, target_brightness);
            }
            LED_CONTROL_LOGI("Brightness set to [%d] %%", target_brightness);
        } break;

        case EVT_TOUCH_SENSOR_SINGLE_PRESS:
        case EVT_BUTTON_SINGLE_PRESS:
        {
            exit_action_led_strip_on(fsm);
            enter_seq_led_strip_off(fsm);

        } break;

        default:
            LED_CONTROL_LOGI("event [%s] not handled in state [%s]",
                    event_router_get_id_name(fsm->event.id),
                    state_name[fsm->state.curr]);
            break;
    }

}

static void led_control_fsm_init(led_control_fsm_t *fsm)
{
    LED_CONTROL_LOGI("Initializing LED Control FSM");
    memset(&fsm->iface, 0, sizeof(led_control_iface_t));
    fsm->state.last = STATE_INVALID;

    // LED Animation default configuration
    fsm->iface.on_animation = LED_ANIMATION_LED_FORWARD_ON;
    fsm->iface.off_animation = LED_ANIMATION_LED_BACKWARD_OFF;
    fsm->iface.dim_up_level = DIM_LEVEL_10;  
    fsm->iface.dim_down_level = DIM_LEVEL_1; 
    fsm->iface.current_dim_level = fsm->iface.dim_up_level;
    fsm->iface.animation_speed = LED_ANIMATION_SPEED; // ms

    // only strip 1 and strip 4 are used
    fsm->iface.strips[LED_STRIP_1].led_count = LED1_STRIP_LED_COUNT;
    fsm->iface.strips[LED_STRIP_2].led_count = LED2_STRIP_LED_COUNT;
    fsm->iface.strips[LED_STRIP_3].led_count = LED3_STRIP_LED_COUNT;
    fsm->iface.strips[LED_STRIP_4].led_count = LED4_STRIP_LED_COUNT;

    // define the strips enabled
    fsm->iface.strips[LED_STRIP_1].enable = LED1_STRIP_ENABLE;
    fsm->iface.strips[LED_STRIP_2].enable = LED2_STRIP_ENABLE;
    fsm->iface.strips[LED_STRIP_3].enable = LED3_STRIP_ENABLE;
    fsm->iface.strips[LED_STRIP_4].enable = LED4_STRIP_ENABLE;

    // initialize the strips
    for (int i = LED_STRIP_1; i < LED_STRIPn; i++)
    {
        uint16_t rmt_channels[] = {LED1_RMT_TX_CHANNEL, LED2_RMT_TX_CHANNEL, LED3_RMT_TX_CHANNEL, LED4_RMT_TX_CHANNEL};
        uint16_t gpios[] = {LED1_RMT_TX_GPIO, LED2_RMT_TX_GPIO, LED3_RMT_TX_GPIO, LED4_RMT_TX_GPIO};

        if (fsm->iface.strips[i].enable)
        {
            fsm->iface.strips[i].control = led_strip_init(rmt_channels[i], gpios[i], fsm->iface.strips[i].led_count, LED_TYPE);
            if (!fsm->iface.strips[i].control) {
                LED_CONTROL_LOGI("LED strip [%d] initialization failed", i + 1);
                return;
            }
            else {
                LED_CONTROL_LOGI("LED strip [%d] Initialized", i + 1);
            }
        }
    }

    led_control_print_info(fsm);

    // default entry state
    enter_seq_led_strip_off(fsm);
}


static esp_err_t led_control_read_event(led_control_fsm_t *fsm)
{
    if (xQueueReceive(led_control_evt_queue, &fsm->event, portMAX_DELAY) == pdPASS)
    {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t led_control_write_event(event_t *event)
{
    if (xQueueSend(led_control_evt_queue, event, (TickType_t)10) == pdPASS) {
        return ESP_OK;
    }
    else {
        LED_CONTROL_LOGI("event [%s] lost", event_router_get_id_name(event->id));
        return ESP_FAIL;
    }
}

static esp_err_t led_control_process_event(led_control_fsm_t *fsm)
{
    switch (fsm->state.curr)
    {
        case STATE_LED_CONTROL_OFF:
            on_state_led_strip_off(fsm);
            break;

        case STATE_LED_CONTROL_ON:
            on_state_led_strip_on(fsm);
            break;

    default:
        break;
    }
    return ESP_OK;
}

static void led_control_print_info(led_control_fsm_t *fsm)
{
    LED_CONTROL_LOGI("ON Animation Type   : %d", fsm->iface.on_animation);
    LED_CONTROL_LOGI("OFF Animation Type  : %d", fsm->iface.off_animation);
    LED_CONTROL_LOGI("Strip1 led count    : %d", fsm->iface.strips[LED_STRIP_1].led_count);
    LED_CONTROL_LOGI("Strip4 led count    : %d", fsm->iface.strips[LED_STRIP_4].led_count);
    LED_CONTROL_LOGI("Dim Up Level        : %d", fsm->iface.dim_up_level);
    LED_CONTROL_LOGI("Dim Down Level      : %d", fsm->iface.dim_down_level);
    LED_CONTROL_LOGI("Animation speed     : %d", fsm->iface.animation_speed);
}


/**
 * @brief Task to handle button events
 * 
 * @param arg 
 */
void led_control_task(void* arg)
{
    LED_CONTROL_LOGI("Task started");
    led_control_fsm_t* fsm = malloc(sizeof(led_control_fsm_t)); 
    if (fsm == NULL) {
        LED_CONTROL_LOGE("Failed to allocate memory for FSM");
        vTaskDelete(NULL);
    }

    led_control_fsm_init(fsm);

    for (;;) {

        if(led_control_read_event(fsm) == ESP_OK) {
            led_control_process_event(fsm);
        }
        else {
            LED_CONTROL_LOGE("Failed to read event");
        }

    }
}

/**
 * @brief Initialize button driver
 * 
 */
void led_control_init(void)
{
    led_control_evt_queue = xQueueCreate(10, sizeof(event_t));
    xTaskCreate(led_control_task, "led_control_task", 2048*4, NULL, 2, NULL);
    LED_CONTROL_LOGI("Initialized");
}

