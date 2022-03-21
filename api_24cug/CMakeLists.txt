cmake_minimum_required(VERSION 3.1)

project(
    api_compensation_24cug
    VERSION 1.0
    LANGUAGES CXX
)

include_directories(/usr/include/libusb-1.0)
include_directories(/usr/include)
# include_directories(/home/kingdra/Projetos/qtcam/src/v4l2headers/include)

add_executable(
    api_compensation_24cug
    api_compensation_24cug.cpp
    uvccamera.cpp
    see3cam_24cug.cpp
    v4l2-api.cpp
)

target_link_libraries(api_compensation_24cug usb-1.0 udev v4l2)