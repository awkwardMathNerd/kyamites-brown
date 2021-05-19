
# set(NRF_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "NCS root directory")

zephyr_include_directories(include)

add_subdirectory(subsys)
add_subdirectory(drivers)
