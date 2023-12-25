idf_component_register(SRCS
                        "main.c"
                        "esp32_s3_driver.c"
                        "task_bme680.c"
                        "task_mqtt.c"
                        "net_interface.c"
                        "utils/struct/struct.c"
                        "utils/struct/struct_endian.c"
                        "utils/crc/crc16.c"
                        "../ic-drivers/platform/platform.c"
                        "../ic-drivers/device/device.c"
                        "../ic-drivers/bme68x/bme68x.c"
                        "lib/libalgobsec.a"
                        "bsec_integration/bsec_integration.c"
                        "bme680_iaq_33v_3s_4d/bsec_iaq.c"
                       INCLUDE_DIRS
                        "../ic-drivers/"
                        "../ic-drivers/platform/"
                        "../ic-drivers/device/"
                        "lib"
                        "inc"
                        "utils"
                        "bsec_integration"
                        "bme680_iaq_33v_3s_4d")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

add_compile_definitions(PLATFORM_TICK_COUNT_TYPE=int64_t)

set(LIBS algobsec)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${LIBS}.a)

target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})

set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})

set(COMPONENT_REQUIRES fatfs vfs)
