# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#idf_component_register(SRCS "../connect.c" "stdin_out.c"
#        INCLUDE_DIRS "include"
#        REQUIRES "tcpip_adapter")

project(i2c)
