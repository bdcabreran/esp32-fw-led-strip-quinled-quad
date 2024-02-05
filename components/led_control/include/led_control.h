/**
 * @file led_control.h
 * @author Bayron Cabrera (bayron.nanez@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "event_router.h"
#include "esp_err.h"
#include "stdbool.h"
#include "esp_timer.h"
#include "time.h"
#include "led_strip.h"

typedef enum
{
    STATE_INVALID = 0, 
    STATE_LED_CONTROL_OFF,
    STATE_LED_CONTROL_ON,
    STATE_LAST
}led_control_state_t;

typedef enum
{
    LED_STRIP_1,
    LED_STRIP_2,
    LED_STRIP_3,
    LED_STRIP_4,
    LED_STRIPn,
}led_strip_channel_t;

typedef enum 
{
    DIM_LEVEL_INVALID = 0,
    DIM_LEVEL_0,
    DIM_LEVEL_1,
    DIM_LEVEL_2,
    DIM_LEVEL_3,
    DIM_LEVEL_4,
    DIM_LEVEL_5,
    DIM_LEVEL_6,
    DIM_LEVEL_7,
    DIM_LEVEL_8,
    DIM_LEVEL_9,
    DIM_LEVEL_10,
    DIM_LEVEL_LAST

}dim_level_t;

typedef enum 
{
    LED_ANIMATION_INVALID = 0,
    LED_ANIMATION_LED_FORWARD_ON,   // turn on the leds from the first to the last
    LED_ANIMATION_LED_BACKWARD_OFF, // turn off the leds from the last to the first
    LED_ANIMATION_LAST,
}led_animation_t;

typedef struct {
    led_strip_t *control; // Pointer to control interface of the LED strip
    uint16_t led_count;   // Number of LEDs in the strip
} led_strip_control_t;

typedef struct {
    led_animation_t on_animation;      // Animation when LED is turned on
    led_animation_t off_animation;     // Animation when LED is turned off
    uint16_t animation_speed;          // Speed of animation
    dim_level_t dim_up_level;          // Dimming level when brightening
    dim_level_t dim_down_level;        // Dimming level when dimming
    dim_level_t current_dim_level;     // Current dim level
    led_strip_control_t strips[LED_STRIPn];     // Array of LED strips
} led_control_iface_t;

typedef struct 
{
    led_control_state_t curr;
    led_control_state_t last;
}led_control_st_t;

typedef struct
{
    led_control_st_t state;
    led_control_iface_t iface;
    event_t event;
}led_control_fsm_t;


void led_control_init(void);
esp_err_t led_control_write_event(event_t *event);


#endif