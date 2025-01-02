# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add boards directory
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Add additional conf file directory
set(EXTRA_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/overlay_hostap.conf)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(simple_osc)

# Compile with hardware float
zephyr_cc_option(-mfloat-abi=hard)

# Set library path for dependents
set(LIBDEPS_PATH "libdeps")

# Create variables for library dependents
set(LIBMAPPER_SOURCE_PATH ${LIBDEPS_PATH}/libmapper-nxp/src)
set(LIBMAPPER_INCLUDE_PATH ${LIBDEPS_PATH}/libmapper-nxp/src)
# Sensor Fusion
set(SENSOR_FUSION_SOURCE_PATH ${LIBDEPS_PATH}/IMU_Sensor_Fusion)
set(SENSOR_FUSION_INCLUDE_PATH ${LIBDEPS_PATH}/IMU_Sensor_Fusion)
# Puara Gestures
# set(PUARA_GESTURES_ROOT ${LIBDEPS_PATH}/puara-gestures)
# set(PUARA_GESTURES_SOURCE_PATH ${PUARA_GESTURES_ROOT}/src)
# set(PUARA_GESTURES_INCLUDE_PATH ${PUARA_GESTURES_ROOT}/include)

# Target repositories in libdeps sources
FILE(GLOB liblo_sources 
    ${LIBMAPPER_SOURCE_PATH}/lo/*.c
    ${LIBMAPPER_SOURCE_PATH}/compat/*.c
    ${LIBMAPPER_SOURCE_PATH}/zlib/*.c
)

FILE(GLOB sensor_fusion_sources
    ${SENSOR_FUSION_SOURCE_PATH}/*.c*
)

# FILE(GLOB puara_gesture_sources
#     ${PUARA_GESTURES_SOURCE_PATH}/*.c*
# )

# Target app sources
FILE(GLOB app_sources src/*.c*)

target_sources(app PRIVATE 
    ${liblo_sources}
    ${sensor_fusion_sources}
    # ${puara_gesture_sources}
    ${app_sources}
)

# add_executable(crc32 crc32.c)
target_include_directories(app PRIVATE 
    ${LIBMAPPER_INCLUDE_PATH}
    ${SENSOR_FUSION_INCLUDE_PATH}
    # ${PUARA_GESTURES_INCLUDE_PATH}
)