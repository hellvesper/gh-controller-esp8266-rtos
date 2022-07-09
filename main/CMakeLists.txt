set(COMPONENT_SRCS "user_main.c" "../protocol_examples_common/connect.c" "../protocol_examples_common/stdin_out.c"
        INCLUDE_DIRS "../protocol_examples_common/include"
        REQUIRES "tcpip_adapter")

register_component()
