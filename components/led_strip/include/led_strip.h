

// #pragma once

// #ifdef __cplusplus
// extern "C" {
// #endif

#ifndef LED_STRIP_H
#define LED_STRIP_H

#include "esp_err.h"

// Enum for LED strip types
typedef enum {
    LED_STRIP_WS2812,
    LED_STRIP_WS2811_LOW_SPEED,
    LED_STRIP_WS2811_HIGH_SPEED,
    LED_STRIP_WS2812B
} led_strip_type_t;

/**
* @brief LED Strip Type
*
*/
typedef struct led_strip_s led_strip_t;

/**
* @brief LED Strip Device Type
*
*/
typedef void *led_strip_dev_t;

/**
* @brief Declare of LED Strip Type
*
*/
struct led_strip_s {
    /**
    * @brief Set RGB for a specific pixel
    *
    * @param strip: LED strip
    * @param index: index of pixel to set
    * @param red: red part of color
    * @param green: green part of color
    * @param blue: blue part of color
    *
    * @return
    *      - ESP_OK: Set RGB for a specific pixel successfully
    *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
    *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
    */
    esp_err_t (*set_pixel)(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);

    /**
    * @brief Refresh memory colors to LEDs
    *
    * @param strip: LED strip
    * @param timeout_ms: timeout value for refreshing task
    *
    * @return
    *      - ESP_OK: Refresh successfully
    *      - ESP_ERR_TIMEOUT: Refresh failed because of timeout
    *      - ESP_FAIL: Refresh failed because some other error occurred
    *
    * @note:
    *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
    */
    esp_err_t (*refresh)(led_strip_t *strip, uint32_t timeout_ms);

    /**
    * @brief Clear LED strip (turn off all LEDs)
    *
    * @param strip: LED strip
    * @param timeout_ms: timeout value for clearing task
    *
    * @return
    *      - ESP_OK: Clear LEDs successfully
    *      - ESP_ERR_TIMEOUT: Clear LEDs failed because of timeout
    *      - ESP_FAIL: Clear LEDs failed because some other error occurred
    */
    esp_err_t (*clear)(led_strip_t *strip, uint32_t timeout_ms);

    /**
    * @brief Free LED strip resources
    *
    * @param strip: LED strip
    *
    * @return
    *      - ESP_OK: Free resources successfully
    *      - ESP_FAIL: Free resources failed because error occurred
    */
    esp_err_t (*del)(led_strip_t *strip);
};

/**
* @brief LED Strip Configuration Type
*
*/
typedef struct {
    uint32_t max_leds;   /*!< Maximum LEDs in a single strip */
    led_strip_dev_t dev; /*!< LED strip device (e.g. RMT channel, PWM channel, etc) */
} led_strip_config_t;

/**
 * @brief Default configuration for LED strip
 *
 */
#define LED_STRIP_DEFAULT_CONFIG(number, dev_hdl) \
    {                                             \
        .max_leds = number,                       \
        .dev = dev_hdl,                           \
    }

/**
* @brief Install a new ws2812 driver (based on RMT peripheral)
*
* @param config: LED strip configuration
* @return
*      LED strip instance or NULL
*/
led_strip_t *led_strip_new_rmt(const led_strip_config_t *config, led_strip_type_t led_type);

/**
 * @brief Init the RMT peripheral and LED strip configuration.
 *
 * @param[in] channel: RMT peripheral channel number.
 * @param[in] gpio: GPIO number for the RMT data output.
 * @param[in] led_num: number of addressable LEDs.
 * @return
 *      LED strip instance or NULL
 */
led_strip_t * led_strip_init(uint8_t channel, uint8_t gpio, uint16_t led_num, led_strip_type_t led_type);


/**
 * @brief Sets the color of a specific LED in an LED strip.
 * 
 * This function configures the color of a single LED in an LED strip
 * to the specified red, green, and blue (RGB) values. The color is set
 * by specifying the LED's position in the strip (index) and the intensity
 * of red, green, and blue components. Each color intensity is represented
 * by a 32-bit unsigned integer, allowing for a wide range of color combinations.
 * 
 * @param strip A pointer to the led_strip_t structure representing the LED strip.
 * @param index The index of the LED within the strip whose color is to be set. 
 *              Indexing starts at 0.
 * @param red The intensity of the red component of the color (0-255).
 * @param green The intensity of the green component of the color (0-255).
 * @param blue The intensity of the blue component of the color (0-255).
 * 
 * @note The function assumes that the `strip` pointer is valid and that the `index`
 *       is within the bounds of the LED strip length. The behavior is undefined if
 *       these conditions are not met. It is the caller's responsibility to ensure
 *       that memory management is properly handled for the `strip` structure.
 * 
 * @warning This function does not check if the index is out of bounds of the LED array.
 *          Passing an index out of bounds can lead to unpredictable behavior or crashes.
 */
void set_led_color(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);


/**
 * @brief Denit the RMT peripheral.
 *
 * @param[in] strip: LED strip
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t led_strip_denit(led_strip_t *strip);

esp_err_t led_strip_set_all(led_strip_t *strip, uint32_t red, uint32_t green, uint32_t blue);
esp_err_t led_strip_dim(led_strip_t *strip, uint8_t percentage);
esp_err_t led_strip_clear_all(led_strip_t *strip, uint32_t timeout_ms);
esp_err_t led_strip_forward_on_sync(led_strip_t *strips[], uint32_t strip_count, uint32_t red, uint32_t green, uint32_t blue, uint32_t delay_ms);
esp_err_t led_strip_backward_off_sync(led_strip_t *strips[], uint32_t strip_count, uint32_t red, uint32_t green, uint32_t blue, uint32_t delay_ms);
void led_rainbow_chase(led_strip_t *strip, uint16_t start_rgb, uint16_t chase_speed_ms, uint16_t led_number);

#endif