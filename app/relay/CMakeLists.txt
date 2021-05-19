# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(client)

set(MYLIB_DIR ../../mylib)
include(${MYLIB_DIR}/CMakeLists.txt)

target_sources(app PRIVATE src/main.c)