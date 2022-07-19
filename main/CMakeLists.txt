set(COMPONENT_SRCS "user_main.c" "i2c_bus.c" "../protocol_examples_common/connect.c" "../protocol_examples_common/stdin_out.c"
#        "../bme280/bme280.c"
        INCLUDE_DIRS "../protocol_examples_common/include"
#        INCLUDE_DIRS "../bme280/include"
        REQUIRES "tcpip_adapter")

#register_component()
