idf_component_register(SRCS
                        "main.c"
                        "esp32_s3_driver.c"
                        "task_bme680.c"
                        "net_interface.c"
                        "D:/git/ic-drivers/platform/platform.c"
                        "D:/git/ic-drivers/device/device.c"
                        "D:/git/ic-drivers/bme68x/bme68x.c"
                        "lib/libalgobsec.a"
                        "bsec_integration/bsec_integration.c"
                        "bme680_iaq_33v_3s_4d/bsec_iaq.c"
                       INCLUDE_DIRS
                        "D:/git/ic-drivers/"
                        "D:/git/ic-drivers/platform/"
                        "D:/git/ic-drivers/device/"
                        "lib"
                        "inc"
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
