# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/version.cmake)


add_definitions(
"-DLV_LVGL_H_INCLUDE_SIMPLE" #"-DLV_COLOR_16_SWAP=1"
) 

# Add newly added components to one of these lines:
# 1. Add here if the component is compatible with IDF >= v4.3
set(EXTRA_COMPONENT_DIRS "core/src" "components" "application" "components/lvgl" "components/eeprom")
set(INCLUDE_DIRECTORIES "core/inc")  
#set(TEST_COMPONENTS "")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(testeCplus)

#target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable)