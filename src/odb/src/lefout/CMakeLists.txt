# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

add_library(lefout
    lefout.cpp
    3DPDKconfig.cpp       # Add the new 3DPDK config source file
)
find_package(yaml-cpp REQUIRED)
target_include_directories(lefout
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${TCL_INCLUDE_PATH}
)

target_link_libraries(lefout
    db
    utl_lib
    Boost::boost
    yaml-cpp
)

set_target_properties(lefout
  PROPERTIES
    # python requirement
    POSITION_INDEPENDENT_CODE ON
)