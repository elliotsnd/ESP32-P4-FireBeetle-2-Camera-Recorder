cmake_minimum_required(VERSION 3.16)

# Add local esp-video-components with IMX708 driver and V4L2-style video framework
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-video-components/esp_cam_sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-video-components/esp_sccb_intf
    ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-video-components/esp_video
    ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-video-components/esp_ipa
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_hw_test)
