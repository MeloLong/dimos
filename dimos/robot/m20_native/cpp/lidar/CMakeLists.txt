cmake_minimum_required(VERSION 3.14)
project(m20_lidar_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

add_executable(m20_lidar_native main.cpp)

target_include_directories(m20_lidar_native PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../hardware/sensors/lidar/common
)

target_link_libraries(m20_lidar_native PRIVATE
  ${LCM_LIBRARIES}
)

target_link_directories(m20_lidar_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS m20_lidar_native DESTINATION bin)
