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
    src/ConsoleUI.cpp
)

# --- SO_REUSEADDR on VRPN's listening sockets -----------------------------
# VRPN binds its server port without SO_REUSEADDR (except on Android), so a
# restart inside the TIME-WAIT window of a client that was connected fails to
# bind and leaves the connection permanently BROKEN. libvrpn is a static
# archive, so the GNU/LLVM linker's --wrap lets us set the option on VRPN's
# own bind() calls without patching VRPN. See src/net_reuseaddr.cpp.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-Wl,--wrap=bind")
check_cxx_source_compiles("int main(){return 0;}" HAVE_LD_WRAP)
unset(CMAKE_REQUIRED_FLAGS)

if (HAVE_LD_WRAP)
    list(APPEND SOURCES src/net_reuseaddr.cpp src/net_client_bounds.cpp)
else()
    # Apple's ld and other linkers have no --wrap. The bridge still recovers,
    # it just has to wait out the TIME-WAIT window (BridgeVicon retries the
    # bind on a backoff and says so).
    message(STATUS "Linker has no --wrap: relying on bind retries for "
                   "TIME-WAIT recovery instead of SO_REUSEADDR.")
endif()

# Add executable
add_executable(vicon_bridge ${SOURCES})

if (HAVE_LD_WRAP)
    set_target_properties(vicon_bridge PROPERTIES LINK_FLAGS "-Wl,--wrap=bind,--wrap=accept,--wrap=listen")
endif()

# Link libraries
target_link_libraries(vicon_bridge PRIVATE
    ${VRPN_LIBRARY}
    ${QUAT_LIBRARY}
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)
