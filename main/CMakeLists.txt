idf_component_register(SRCS "gatts_demo.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio nvs_flash bt)