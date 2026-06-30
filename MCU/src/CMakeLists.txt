# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# To build the main app, uncomment this:
# idf_component_register(SRCS "main.cpp" "app_services.cpp" "motion_task.cpp"
#                     INCLUDE_DIRS "." "../include"
#                     REQUIRES log esp_driver_gpio)

# To build the MotorTest:
idf_component_register(SRCS "../test/MotorTest.cpp"
                    INCLUDE_DIRS "." "../include"
                    REQUIRES log esp_driver_gpio)
