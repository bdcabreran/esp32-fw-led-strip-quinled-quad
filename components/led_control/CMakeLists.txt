idf_component_register(SRCS "led_control.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "driver" "led_strip" "event_router" "button" 
                    )
