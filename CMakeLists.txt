# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(ACS800ModbusRTU LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(ACS800ModbusRTU SHARED src/acs800_reta01.c src/acs800_reta01.h)

if(WIN32)
  target_compile_definitions(ACS800ModbusRTU PRIVATE ACS800_DLL_EXPORTS)
endif()

set_target_properties(ACS800ModbusRTU PROPERTIES
  PREFIX ""
  OUTPUT_NAME "ACS800ModbusRTU"
)
