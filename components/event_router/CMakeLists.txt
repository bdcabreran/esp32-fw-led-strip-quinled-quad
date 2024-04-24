idf_component_register(SRCS "event_router.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "driver" "button" "led_control"
                    )
