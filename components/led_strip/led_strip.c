/**
 * @file led_strip_rmt_ws2812.c
 * @brief Implementation of the LED strip control using RMT for WS2812 LEDs.
 *
 * This file contains the implementation of the LED strip control using RMT (Remote Control) for WS2812 LEDs.
 * It provides functions to initialize the timing parameters, handle LED strip types, and control the LED strip.
 */


#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "led_strip.h"
#include "driver/rmt.h"


static const char *TAG = "led_strip";
#define STRIP_CHECK(a, str, goto_tag, ret_value, ...)                             \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)


// Timing definitions for various LED types

// WS2811 LOW SPEED
#define WS2811_LOW_SPEED_T0H_NS (500)
#define WS2811_LOW_SPEED_T1H_NS (1200)
#define WS2811_LOW_SPEED_T0L_NS (2000)
#define WS2811_LOW_SPEED_T1L_NS (1300)
#define WS2811_LOW_SPEED_RESET_US (280)

// WS2811 HIGH SPEED
#define WS2811_HIGH_SPEED_T0H_NS (250)
#define WS2811_HIGH_SPEED_T1H_NS (600)
#define WS2811_HIGH_SPEED_T0L_NS (1000)
#define WS2811_HIGH_SPEED_T1L_NS (650)
#define WS2811_HIGH_SPEED_RESET_US (280)

// WS2812B
#define WS2812B_T0H_NS (220) 
#define WS2812B_T1H_NS (580) 
#define WS2812B_T0L_NS (580) 
#define WS2812B_T1L_NS (420) 
#define WS2812B_RESET_US (280)

// WS2812
#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (800)
#define WS2812_T1H_NS (700)
#define WS2812_T1L_NS (600)
#define WS2812_RESET_US (280)


// Structure to hold LED timing information
typedef struct {
    uint32_t t0h_ticks;
    uint32_t t1h_ticks;
    uint32_t t0l_ticks;
    uint32_t t1l_ticks;
    uint32_t reset_ticks;
} led_timing_t;

// Global lookup table for RMT channel to led_timing_t mapping
static led_timing_t *rmt_channel_timings[RMT_CHANNEL_MAX] = {0};


typedef struct {
    led_strip_t parent;
    rmt_channel_t rmt_channel;
    uint32_t strip_len;
    led_timing_t timing;  
    uint8_t *buffer;           // Current (possibly dimmed) colors
    uint8_t *original_buffer; // Original color values

} led_controller_t;


/**
 * @brief Initializes the timing parameters for the LED strip based on the selected type.
 *
 * This function calculates the timing parameters for the LED strip based on the selected type and the counter clock frequency.
 *
 * @param type The type of LED strip.
 * @param counter_clk_hz The counter clock frequency in Hz.
 * @param timing Pointer to the structure to store the calculated timing parameters.
 */
static void init_timing(led_strip_type_t type, uint32_t counter_clk_hz, led_timing_t *timing) {
    float ratio = (float)counter_clk_hz / 1e9;

    switch (type) {
        case LED_STRIP_WS2811_LOW_SPEED:
            timing->t0h_ticks = (uint32_t)(ratio * WS2811_LOW_SPEED_T0H_NS);
            timing->t0l_ticks = (uint32_t)(ratio * WS2811_LOW_SPEED_T0L_NS);
            timing->t1h_ticks = (uint32_t)(ratio * WS2811_LOW_SPEED_T1H_NS);
            timing->t1l_ticks = (uint32_t)(ratio * WS2811_LOW_SPEED_T1L_NS);
            break;
        case LED_STRIP_WS2811_HIGH_SPEED:
            timing->t0h_ticks = (uint32_t)(ratio * WS2811_HIGH_SPEED_T0H_NS);
            timing->t0l_ticks = (uint32_t)(ratio * WS2811_HIGH_SPEED_T0L_NS);
            timing->t1h_ticks = (uint32_t)(ratio * WS2811_HIGH_SPEED_T1H_NS);
            timing->t1l_ticks = (uint32_t)(ratio * WS2811_HIGH_SPEED_T1L_NS);
            break;
        case LED_STRIP_WS2812B:
            timing->t0h_ticks = (uint32_t)(ratio * WS2812B_T0H_NS);
            timing->t0l_ticks = (uint32_t)(ratio * WS2812B_T0L_NS);
            timing->t1h_ticks = (uint32_t)(ratio * WS2812B_T1H_NS);
            timing->t1l_ticks = (uint32_t)(ratio * WS2812B_T1L_NS);
            break;
        case LED_STRIP_WS2812:
        default:
            timing->t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
            timing->t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
            timing->t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
            timing->t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);
            break;
    }
}


/**
 * @brief Conver RGB data to RMT format.
 *
 * @note For WS2812, R,G,B each contains 256 different choices (i.e. uint8_t)
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] src_size: size of source data
 * @param[in] wanted_num: number of RMT items that want to get
 * @param[out] translated_size: number of source data that got converted
 * @param[out] item_num: number of RMT items which are converted from source data
 */

static void IRAM_ATTR led_controller_convert_to_rmt(const void *src, rmt_item32_t *dest, size_t src_size,
                                            size_t wanted_num, size_t *translated_size, size_t *item_num,
                                            const led_timing_t *timing) {
    if (src == NULL || dest == NULL || timing == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }

    const rmt_item32_t bit0 = {{{ timing->t0h_ticks, 1, timing->t0l_ticks, 0 }}}; // Logical 0
    const rmt_item32_t bit1 = {{{ timing->t1h_ticks, 1, timing->t1l_ticks, 0 }}}; // Logical 1

    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            if (*psrc & (1 << (7 - i))) {
                pdest->val = bit1.val;
            } else {
                pdest->val = bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}



static void IRAM_ATTR led_rmt_adapter_channel_0(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_0];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_1(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_1];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_2(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_2];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_3(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_3];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_4(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_4];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_5(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_5];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_6(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_6];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}

static void IRAM_ATTR led_rmt_adapter_channel_7(const void *src, rmt_item32_t *dest, size_t src_size,
                                                   size_t wanted_num, size_t *translated_size, size_t *item_num) {
    led_timing_t *timing = rmt_channel_timings[RMT_CHANNEL_7];
    led_controller_convert_to_rmt(src, dest, src_size, wanted_num, translated_size, item_num, timing);
}



static esp_err_t led_controller_set_pixel(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    esp_err_t ret = ESP_OK;
    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    STRIP_CHECK(index < led_controller->strip_len, "index out of the maximum number of leds", err, ESP_ERR_INVALID_ARG);
    uint32_t start = index * 3;
    // In thr order of GRB
    led_controller->buffer[start + 0] = green & 0xFF;
    led_controller->buffer[start + 1] = red & 0xFF;
    led_controller->buffer[start + 2] = blue & 0xFF;

    // Original code for setting pixel
    // Additionally, store the color in original_buffer
    led_controller->original_buffer[start + 0] = green & 0xFF;
    led_controller->original_buffer[start + 1] = red & 0xFF;
    led_controller->original_buffer[start + 2] = blue & 0xFF;

    return ESP_OK;
err:
    return ret;
}

static esp_err_t led_controller_refresh(led_strip_t *strip, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    STRIP_CHECK(rmt_write_sample(led_controller->rmt_channel, led_controller->buffer, led_controller->strip_len * 3, true) == ESP_OK,
                "transmit RMT samples failed", err, ESP_FAIL);
    return rmt_wait_tx_done(led_controller->rmt_channel, pdMS_TO_TICKS(timeout_ms));
err:
    return ret;
}

static esp_err_t led_controller_clear(led_strip_t *strip, uint32_t timeout_ms)
{
    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    // Write zero to turn off all leds
    memset(led_controller->buffer, 0, led_controller->strip_len * 3);
    return led_controller_refresh(strip, timeout_ms);
}

static esp_err_t led_controller_del(led_strip_t *strip)
{
    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    rmt_channel_timings[led_controller->rmt_channel] = NULL;
    free(led_controller);
    return ESP_OK;
}

led_strip_t *led_strip_new_rmt(const led_strip_config_t *config, led_strip_type_t led_type)
{
    led_strip_t *ret = NULL;
    STRIP_CHECK(config, "configuration can't be null", err, NULL);

    // 24 bits per led
    uint32_t led_controller_size = sizeof(led_controller_t) + (config->max_leds * 3)*2;
    led_controller_t *led_controller = calloc(1, led_controller_size);

    STRIP_CHECK(led_controller, "request memory for led_controller failed", err, NULL);

    // Assign the buffer pointer to immediately after the led_controller struct
    led_controller->buffer = (uint8_t *)led_controller + sizeof(led_controller_t);

    // Assign the original_buffer pointer to immediately after the buffer
    led_controller->original_buffer = led_controller->buffer + (config->max_leds * 3);


    uint32_t counter_clk_hz = 0;
    STRIP_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev, &counter_clk_hz) == ESP_OK,
                "get rmt counter clock failed", err, NULL);

    init_timing(led_type, counter_clk_hz, &led_controller->timing);

    // During initialization, after calculating the timing information
    led_controller->rmt_channel = (rmt_channel_t)config->dev;
    rmt_channel_timings[led_controller->rmt_channel] = &led_controller->timing;
    
    led_controller->strip_len = config->max_leds;

    ESP_LOGI(TAG, "Initializing led_controller strip on RMT channel %d", led_controller->rmt_channel);
    led_controller->parent.set_pixel = led_controller_set_pixel;
    led_controller->parent.refresh = led_controller_refresh;
    led_controller->parent.clear = led_controller_clear;
    led_controller->parent.del = led_controller_del;

    // Set the RMT translator based on the channel
    switch (led_controller->rmt_channel)
    {
    case RMT_CHANNEL_0:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_0);
        break;
    case RMT_CHANNEL_1:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_1);
        break;
    case RMT_CHANNEL_2:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_2);
        break;
    case RMT_CHANNEL_3:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_3);
        break;
    case RMT_CHANNEL_4:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_4);
        break;
    case RMT_CHANNEL_5:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_5);
        break;
    case RMT_CHANNEL_6:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_6);
        break;
    case RMT_CHANNEL_7:
        rmt_translator_init((rmt_channel_t)config->dev, led_rmt_adapter_channel_7);
        break;
    default:
        break;
    }

    return &led_controller->parent;
err:
    return ret;
}

led_strip_t * led_strip_init(uint8_t channel, uint8_t gpio, uint16_t led_num, led_strip_type_t led_type)
{

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio, channel);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // Install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(led_num, (led_strip_dev_t)config.channel);
    led_strip_t *pStrip = led_strip_new_rmt(&strip_config, led_type);


    if ( !pStrip ) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
        return NULL;
    }

    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(pStrip->clear(pStrip, 100));

    return pStrip;
}

esp_err_t led_strip_denit(led_strip_t *strip)
{

    if (!strip) {
        // Handle null pointer if necessary
        return ESP_ERR_INVALID_ARG;
    }

    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    // Uninstall the RMT driver
    esp_err_t err = rmt_driver_uninstall(led_controller->rmt_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT driver uninstall failed");
        return err;
    }

    // Clear the timing information for this channel
    rmt_channel_timings[led_controller->rmt_channel] = NULL;

    // Free the allocated memory for the led_controller structure and its buffer
    free(led_controller);

    return ESP_OK;
}


void set_led_color(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue) {
    // Set the color of the LED at the specified index
    esp_err_t ret = strip->set_pixel(strip, index, red, green, blue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set LED Color failed");
    }

    // Refresh the strip to apply the changes
    ret = strip->refresh(strip, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Refresh LED Color failed");
    }
}


esp_err_t led_strip_set_all(led_strip_t *strip, uint32_t red, uint32_t green, uint32_t blue) {
    if (!strip) {
        return ESP_ERR_INVALID_ARG;
    }

    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    for (uint32_t i = 0; i < led_controller->strip_len; i++) {
        led_controller_set_pixel(strip, i, red, green, blue);
    }

    // You might want to refresh the strip here or leave it to the user to call refresh separately.
    return strip->refresh(strip, 100);
}

esp_err_t led_strip_dim(led_strip_t *strip, uint8_t percentage) {
    if (!strip || percentage > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    for (uint32_t i = 0; i < led_controller->strip_len * 3; i++) {
        // Apply dimming based on the original color values
        uint32_t original_color_value = led_controller->original_buffer[i];
        led_controller->buffer[i] = (original_color_value * percentage) / 100;
    }

    return strip->refresh(strip, 100);
}


// Adjusted function prototype
esp_err_t led_strip_dim_smooth(led_strip_t *strips[], uint32_t strip_count, uint8_t current_percentage, uint8_t target_percentage, uint32_t transition_time_ms) {
    // Validate inputs
    if (!strips || strip_count == 0 || target_percentage > 100 || current_percentage > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Dimming from %d%% to %d%% over %dms", current_percentage, target_percentage, transition_time_ms);

    const uint32_t steps = 20; // Number of steps in the transition
    const uint32_t delay_per_step_ms = transition_time_ms / steps;
    int32_t step_size = (target_percentage - current_percentage) / (int32_t)steps;

    for (uint32_t step = 0; step <= steps; step++) {
        // Calculate new brightness level for this step
        uint8_t new_percentage = current_percentage + (step_size * step);

        for (uint32_t strip_index = 0; strip_index < strip_count; strip_index++) {
            if (strips[strip_index] == NULL) continue; // Skip if strip is NULL

            led_controller_t *led_controller = __containerof(strips[strip_index], led_controller_t, parent);
            for (uint32_t i = 0; i < led_controller->strip_len * 3; i++) {
                uint32_t original_color_value = led_controller->original_buffer[i];
                led_controller->buffer[i] = (original_color_value * new_percentage) / 100;
            }
            
            // Refresh the strip to apply the new brightness
            strips[strip_index]->refresh(strips[strip_index], 100);
        }

        // Delay to create smooth transition
        vTaskDelay(pdMS_TO_TICKS(delay_per_step_ms));
    }

    // Ensure the final brightness is set to the target value for each strip
    for (uint32_t strip_index = 0; strip_index < strip_count; strip_index++) {
        if (strips[strip_index] == NULL) continue; // Skip if strip is NULL

        led_controller_t *led_controller = __containerof(strips[strip_index], led_controller_t, parent);
        for (uint32_t i = 0; i < led_controller->strip_len * 3; i++) {
            uint32_t original_color_value = led_controller->original_buffer[i];
            led_controller->buffer[i] = (original_color_value * target_percentage) / 100;
        }
        strips[strip_index]->refresh(strips[strip_index], 100);
    }

    return ESP_OK;
}


// This function is just a wrapper for the existing clear method.
esp_err_t led_strip_clear_all(led_strip_t *strip, uint32_t timeout_ms) {
    if (!strip) {
        return ESP_ERR_INVALID_ARG;
    }
    // Call the existing clear function.
    return strip->clear(strip, timeout_ms);
}

esp_err_t led_strip_forward_on(led_strip_t *strip, uint32_t red, uint32_t green, uint32_t blue, uint32_t delay_ms) {
    if (!strip) return ESP_ERR_INVALID_ARG;
    
    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    for (uint32_t i = 0; i < led_controller->strip_len; i++) {
        strip->set_pixel(strip, i, red, green, blue);
        strip->refresh(strip, delay_ms); // Apply the change and wait
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay to create animation effect
    }
    return ESP_OK;
}

esp_err_t led_strip_backward_off(led_strip_t *strip, uint32_t delay_ms) {
    if (!strip) return ESP_ERR_INVALID_ARG;
    
    led_controller_t *led_controller = __containerof(strip, led_controller_t, parent);
    for (int32_t i = led_controller->strip_len - 1; i >= 0; i--) {
        strip->set_pixel(strip, i, 0, 0, 0); // Turn off the LED
        strip->refresh(strip, delay_ms); // Apply the change and wait
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay to create animation effect
    }
    return ESP_OK;
}



esp_err_t led_strip_forward_on_sync(led_strip_t *strips[], uint32_t strip_count, uint32_t red, uint32_t green, uint32_t blue, uint32_t delay_ms) {
    // Validate input parameters
    if (!strips || strip_count == 0) return ESP_ERR_INVALID_ARG;
    
    // Find the maximum length among the strips to ensure we loop through all LEDs
    uint32_t max_len = 0;
    for (uint32_t s = 0; s < strip_count; s++) {
        if (strips[s] != NULL) {
            led_controller_t *led_controller = __containerof(strips[s], led_controller_t, parent);
            if (led_controller->strip_len > max_len) {
                max_len = led_controller->strip_len;
            }
        }
    }

    // Loop through each LED position
    for (uint32_t i = 0; i < max_len; i++) {
        for (uint32_t s = 0; s < strip_count; s++) {
            if (strips[s] != NULL) {
                led_controller_t *led_controller = __containerof(strips[s], led_controller_t, parent);
                // Check if the current strip length is greater than the current index
                if (i < led_controller->strip_len) {
                    strips[s]->set_pixel(strips[s], i, red, green, blue);
                    strips[s]->refresh(strips[s], delay_ms); // Apply changes
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay for synchronization
    }
    return ESP_OK;
}


esp_err_t led_strip_backward_off_sync(led_strip_t *strips[], uint32_t strip_count, uint32_t red, uint32_t green, uint32_t blue, uint32_t delay_ms) {
    // Validate input parameters
    if (!strips || strip_count == 0) return ESP_ERR_INVALID_ARG;
    
    // Find the maximum length among the strips to ensure we loop through all LEDs
    uint32_t max_len = 0;
    for (uint32_t s = 0; s < strip_count; s++) {
        if (strips[s] != NULL) {
            led_controller_t *led_controller = __containerof(strips[s], led_controller_t, parent);
            if (led_controller->strip_len > max_len) {
                max_len = led_controller->strip_len;
            }
        }
    }

    // Loop through each LED position in reverse order
    for (int32_t i = max_len - 1; i >= 0; i--) {
        for (uint32_t s = 0; s < strip_count; s++) {
            if (strips[s] != NULL) {
                led_controller_t *led_controller = __containerof(strips[s], led_controller_t, parent);
                // Check if the current index is within the strip length
                if (i < led_controller->strip_len) {
                    // Turn off the LED at the current position
                    strips[s]->set_pixel(strips[s], i, red, green, blue);
                    strips[s]->refresh(strips[s], delay_ms); // Apply changes
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay for synchronization
    }
    return ESP_OK;
}


/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 * HSV Color Space: In the HSV model, Hue represents the color type, Saturation represents the intensity of the color, and Value represents the brightness of the color.
 *
 * Hue (H) is measured in degrees, ranging from 0 to 360, where each value corresponds to a color on the color wheel (e.g., 0 or 360 is red, 120 is green, 240 is blue).
 * Saturation (S) is a percentage value from 0 to 100, where 0 means a shade of gray and 100 represents the full color.
 * Value (V) is also a percentage from 0 to 100, where 0 is completely black, and 100 is the brightest and reveals the most of the color.
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

/**
 * @brief Creates a rainbow chase effect on an LED strip.
 * 
 * This function cycles through a rainbow of colors and applies them to the LED strip,
 * creating a "chasing" effect where groups of LEDs light up in sequence along the strip.
 * 
 * @param strip The LED strip object.
 * @param start_rgb The starting hue value for the rainbow effect (0-359 degrees).
 * @param chase_speed_ms The speed of the chase effect, in milliseconds.
 * @param led_number The total number of LEDs in the strip.
 */
void led_rainbow_chase(led_strip_t *strip, uint16_t start_rgb, uint16_t chase_speed_ms, uint16_t led_number) {
    uint32_t red = 0;   
    uint32_t green = 0; 
    uint32_t blue = 0;  
    uint16_t hue = 0;   

    // Loop to create the chase effect in three steps, lighting up every third LED in sequence.
    for (int i = 0; i < 3; i++) {
        // Loop over every third LED starting from 'i' to create a spaced-out chase effect.
        for (int j = i; j < led_number; j += 3) {
            // Calculate the hue value for the current LED based on its position and the starting hue.
            // This creates a gradual rainbow across the strip.
            hue = j * 360 / led_number + start_rgb;

            // Convert the HSV values to RGB values. Here, the saturation is set to 100%,
            // and the value (brightness) is set to 20% for all LEDs.
            led_strip_hsv2rgb(hue, 100, 20, &red, &green, &blue);

            // Set the calculated RGB color to the current LED.
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
        }
        // Send the updated color information to the LED strip to display the changes.
        ESP_ERROR_CHECK(strip->refresh(strip, 100));

        // Wait for the specified chase speed duration before continuing to the next step.
        vTaskDelay(pdMS_TO_TICKS(chase_speed_ms));

        // Clear the strip after each chase step to turn off all LEDs before the next cycle starts.
        strip->clear(strip, 50);

        // Wait again for the specified duration to maintain consistent timing.
        vTaskDelay(pdMS_TO_TICKS(chase_speed_ms));
    }
    // Increment the starting hue by 60 degrees to shift the colors for the next function call.
    start_rgb += 60;
}


#if 0 // Example usage of the LED strip functions (put this in main.c)
/**
 * @brief Task to create a rainbow chase effect on LED1 strip
 *
 */
void led_rainbow_chase_task1(void *pvParameters) {

    led_strip_t *strip1 =  led_strip_init(LED1_RMT_TX_CHANNEL, LED1_RMT_TX_GPIO, LED1_STRIP_LED_NUMBER, LED_STRIP_WS2812);
    if (!strip1) {
        ESP_LOGE("main", "LED strip1 initialization failed");
        return;
    }

    uint16_t start_rgb = 0;
    ESP_LOGI(TAG, "LED1 Rainbow Chase Start");

    while (true) {
        led_rainbow_chase(strip1, start_rgb, LED1_CHASE_SPEED_MS, LED1_STRIP_LED_NUMBER);
    }
}

void app_main(void)
{
    // xTaskCreate(led_rainbow_chase_task1, "LED Rainbow Chase Task 1", 2048, NULL, 5, NULL);
}

#endif 