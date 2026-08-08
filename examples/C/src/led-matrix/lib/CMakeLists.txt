add_library(led_matrix
  src/ws2812.c
  src/led_remapping.c
)

# pico_sdk_import.cmake
if(NOT DEFINED PICO_SDK_PATH)
  set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/pico-sdk" CACHE PATH "Path to the Pico SDK")
endif()

# Download automatically if not present
if(NOT EXISTS ${PICO_SDK_PATH})
  message(STATUS "Cloning pico-sdk into ${PICO_SDK_PATH}")
  execute_process(
    COMMAND git clone -b master https://github.com/raspberrypi/pico-sdk.git ${PICO_SDK_PATH}
  )
endif()

# Pull in Pico SDK (must be after pico_sdk_path is defined)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

pico_generate_pio_header(led_matrix 
  ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio
)

target_include_directories(led_matrix PUBLIC src)

target_link_libraries(led_matrix
  pico_stdlib
  hardware_adc
  hardware_pio
  hardware_pwm
  hardware_spi
  hardware_i2c
)
