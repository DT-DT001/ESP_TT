idf_component_register(SRCS "hello_world_main.c"
                    PRIV_REQUIRES spi_flash driver esp_driver_uart esp_driver_usb_serial_jtag vfs
                    INCLUDE_DIRS "")
