
/* RMT example -- RGB LED Strip
   This example uses the RMT peripheral to control a RGB LED strip.
   Author: Bayron Cabrera
   Email: bayron.nanez@gmail.com
*/

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "button.h"
#include "touch.h"
#include "event_router.h"
#include "led_control.h"



/**
 * @brief Print the startup message to the monitor
 *
 */
void print_startup_message() {
    const char* projectName = "esp32-fw-led-strip-quinled-quad";
    const char* author = "Bayron Cabrera";
    const char* date = "2023-01-03";
    const char* version = "1.0.0";

    printf("Project: %s\n", projectName);
    printf("Author: %s\n", author);
    printf("Date: %s\n", date);
    printf("Version: %s\n", version);

    // Get and print the ESP-IDF version
    const char* idfVersion = esp_get_idf_version();
    printf("ESP-IDF Version: %s\n", idfVersion);
}

/**
 * @brief Main function
 *
 */
void app_main(void)
{
    print_startup_message();
    event_router_init();
    button_init();
    touch_init();
    led_control_init();
}



