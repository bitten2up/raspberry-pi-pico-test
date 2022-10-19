cmake_minimum_required(VERSION 3.13)

add_executable(bittenpico)
target_sources(bittenpico PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(bittenpico PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})
# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(bittenpico PRIVATE pico_stdlib hardware_resets hardware_irq)

pico_add_extra_outputs(bittenpico)