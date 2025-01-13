#idf_component_register(SRCS "src/main.cpp" "src/adc_functions.c" "src/bmi088.c" "src/ds18b20.c" "src/mpu9250.c"
#                       INCLUDE_DIRS "." "inc/.")

# Automatically collect all .c and .cpp files in the src directory
file(GLOB SOURCES "src/*.c" "src/*.cpp")

# Register the component, using the collected source files and include directory
idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS "." "inc")

