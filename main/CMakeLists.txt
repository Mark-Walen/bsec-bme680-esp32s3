idf_component_register(SRCS
                        "main.c"
                        "esp32_s3_driver.c"
                        "D:/git/ic-drivers/device/device.c"
                        "D:/git/ic-drivers/w25qxx/w25qxx.c"
                        "D:/git/ic-drivers/adxl362/adxl362.c"
                       INCLUDE_DIRS
                        "D:/git/ic-drivers/"
                        "D:/git/ic-drivers/device/")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
