px4_add_module(
    MODULE modules__cgtd070
    MAIN cgtd070
    COMPILE_FLAGS
        -fno-exceptions
        -fno-rtti
        -fno-threadsafe-statics
    SRCS
        px4_module/cgtd070_app.cpp
        library/src/cgtd070_core.cpp
        drivers/serial/cgtd070_serial_driver.cpp
    INCLUDES
        library/include
        drivers/serial
        px4_module
)

