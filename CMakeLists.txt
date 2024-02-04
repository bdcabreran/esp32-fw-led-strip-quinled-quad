# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# No longer need to set EXTRA_COMPONENT_DIRS if the component is within the project's components directory
#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/led_strip)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-fw-led-stript-quinled-quad)
