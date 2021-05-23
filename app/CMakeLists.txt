# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pf)

# include the CMakeLists.txt file from the myoslib directory
set(MYOSLIB_DIR ../../myoslib)
include(${MYOSLIB_DIR}/CMakeLists.txt)

# define a list of myoslibs used
list(APPEND myoslib_srcs
)

# compile our source code with myoslibs
#target_sources(app PRIVATE src/main.c ${myoslib_srcs})
target_sources(app PRIVATE src/main.c)