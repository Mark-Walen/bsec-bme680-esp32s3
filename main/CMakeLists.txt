idf_component_register(SRCS
                        "main.c"
                        "esp32_s3_driver.c"
                        "task_adxl362.c"
                        "task_adxl345.c"
                        "D:/git/ic-drivers/platform/platform.c"
                        "D:/git/ic-drivers/device/device.c"
                        "D:/git/ic-drivers/w25qxx/w25qxx.c"
                        "D:/git/ic-drivers/adxl362/adxl362.c"
                        "D:/git/ic-drivers/adxl345/adxl345.c"
                       INCLUDE_DIRS
                        "D:/git/ic-drivers/"
                        "D:/git/ic-drivers/platform/"
                        "D:/git/ic-drivers/device/")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

add_compile_definitions(DEVICE_TICK_COUNT_TYPE=int64_t)
