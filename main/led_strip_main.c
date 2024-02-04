
/* RMT example -- RGB LED Strip

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   Author: Bayron Cabrera
   Email: bayron.nanez@gmail.com
*/


#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"


void print_startup_message() {
    const char* projectName = "esp32-fw-led-stript-quinled-quad";
    const char* author = "Bayron Cabrera";
    const char* date = "2023-01-03";
    const char* version = "1.0.0";

    printf("Project: %s\n", projectName);
    printf("Author: %s\n", author);
    printf("Date: %s\n", date);
    printf("Version: %s\n", version);
}


static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define EXAMPLE_CHASE_SPEED_MS (50)
#define USER_STRIP_LED_NUMBER (28)
#define USER_RMT_TX_GPIO (16)

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
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


void chase_pattern(led_strip_t *strip, uint32_t delay_ms, uint32_t iterations) {
    uint32_t red = 255;   // Set red value to maximum (255)
    uint32_t green = 0;   // Set green value to minimum (0)
    uint32_t blue = 0;    // Set blue value to minimum (0)

    for (uint32_t iter = 0; iter < iterations; iter++) {
        for (int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++) {
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, i, red, green, blue));

            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));

            // Clear the LED for the next cycle
            ESP_ERROR_CHECK(strip->clear(strip, 50));

            // Delay between updates
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

void app_main(void)
{

    print_startup_message();

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(USER_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(USER_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config, LED_STRIP_WS2812);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");
    while (true) {

        // chase_pattern(strip, EXAMPLE_CHASE_SPEED_MS, 1);
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < USER_STRIP_LED_NUMBER; j += 3) {
                // Build RGB values
                hue = j * 360 / USER_STRIP_LED_NUMBER + start_rgb;
                led_strip_hsv2rgb(hue, 100, 2, &red, &green, &blue);
                // Write RGB values to strip driver
                ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            strip->clear(strip, 50);
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}


