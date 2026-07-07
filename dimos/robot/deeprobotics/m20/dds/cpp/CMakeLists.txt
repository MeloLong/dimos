cmake_minimum_required(VERSION 3.14)
project(m20_dds_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# dimos_lcm: generated C++ LCM message headers (sensor_msgs/, nav_msgs/, ...).
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

add_executable(m20_dds_bridge main.cpp)

# drdds/dridl live under /usr/local (installed with the DeepRobotics SDK).
# Include layout mirrors ~/drcloud: -I/usr/local/include lets `drdds/...` and
# `dridl/...` resolve via the /usr/local/include/drdds prefix.
target_include_directories(m20_dds_bridge PRIVATE
  /usr/local/include
  /usr/local/include/drdds
  /usr/local/include/dridl
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(m20_dds_bridge PRIVATE
  /usr/local/lib
  ${LCM_LIBRARY_DIRS}
)

target_link_libraries(m20_dds_bridge PRIVATE
  drdds
  fastrtps
  fastcdr
  pthread
  ${LCM_LIBRARIES}
)

install(TARGETS m20_dds_bridge DESTINATION bin)
