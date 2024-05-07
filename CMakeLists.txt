idf_component_register(SRCS "bme68x_lib.c" "bme68x/bme68x.c"
                    INCLUDE_DIRS "include" "bme68x"
                    REQUIRES esp_timer driver)
