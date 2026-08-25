idf_component_register(SRCS "cc1101.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_spi esp_driver_gpio esp_timer)
