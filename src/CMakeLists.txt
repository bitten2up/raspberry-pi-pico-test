cmake_minimum_required(VERSION 3.13)

add_executable(bittenpico)

target_sources(bittenpico PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/msc_disk.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        ${CMAKE_SOURCE_DIR}/flash_program.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(bittenpico PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(bittenpico PUBLIC pico_stdlib tinyusb_device tinyusb_board)

pico_add_extra_outputs(bittenpico)

# add url via pico_set_program_url
example_auto_set_url(bittenpico)