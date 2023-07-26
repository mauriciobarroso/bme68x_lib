idf_component_register(SRCS "bme68x_lib.c" "bme68x/bme68x.c"
                    INCLUDE_DIRS "include" "bme68x"
                    REQUIRES i2c_bus esp_timer driver i2c_bus)
