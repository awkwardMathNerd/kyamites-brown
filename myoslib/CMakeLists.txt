# CMakeLists.txt for myoslib
# Defines all the targets for each myoslib which can then just be
# added to an application by adding the target name to the
# app target source list

include_directories(${CMAKE_CURRENT_LIST_DIR}/inc)

# hal_mqtt myoslib
list (APPEND hal_mqtt_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/hal_mqtt.c
)

# hal_wifi myoslib
list (APPEND hal_wifi_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/hal_wifi.c
)

# meas_packet
list (APPEND meas_packet_pb_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/pb_common.c
    ${CMAKE_CURRENT_LIST_DIR}/src/pb_decode.c
    ${CMAKE_CURRENT_LIST_DIR}/src/pb_encode.c
    ${CMAKE_CURRENT_LIST_DIR}/src/meas_packet.pb.c
)