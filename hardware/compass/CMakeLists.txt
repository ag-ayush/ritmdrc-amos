cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

set(FIRMWARE_NAME compass)

#set(${FIRMWARE_NAME}_BOARD diecimila)
set(${FIRMWARE_NAME}_BOARD atmega328)
set(${${FIRMWARE_NAME}_BOARD}.upload.protocol arduino)
set(${FIRMWARE_NAME}_SRCS src/compass.cpp)
set(${FIRMWARE_NAME}_PORT /dev/compass)

generate_ros_firmware(${FIRMWARE_NAME})

