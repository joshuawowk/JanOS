idf_component_register(SRCS "subghz.c" "subghz_capture.c" "subghz_decode.c" "subghz_store.c"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_spi esp_driver_gpio esp_driver_rmt esp_timer console
                             freertos nvs_flash fatfs cc1101)
