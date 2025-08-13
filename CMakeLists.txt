cmake_minimum_required(VERSION 3.10)
project(ViconBridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_library(VRPN_LIBRARY NAMES vrpn)
find_library(QUAT_LIBRARY NAMES quat)
find_path(VRPN_INCLUDE_DIR vrpn_Tracker.h)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

if (NOT VRPN_LIBRARY OR NOT QUAT_LIBRARY OR NOT VRPN_INCLUDE_DIR)
    message(FATAL_ERROR "VRPN or Quat library/headers not found!")
endif()

# Include directories
include_directories(${VRPN_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/main.cpp
    src/BridgeVicon.cpp
)

# Add executable
add_executable(vicon_bridge ${SOURCES})

# Link libraries
target_link_libraries(vicon_bridge PRIVATE
    ${VRPN_LIBRARY}
    ${QUAT_LIBRARY}
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)