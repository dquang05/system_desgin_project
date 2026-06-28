# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

#idf_component_register(SRCS ${app_sources})
idf_component_register(SRCS "main.cpp" "app_services.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES log esp_driver_gpio)
