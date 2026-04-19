cmake_minimum_required(VERSION 3.16)
project(IMUBLE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libraries" FORCE)
# set(SIMPLEBLE_TEST OFF CACHE BOOL "Disable tests" FORCE)
# set(SIMPLEBLE_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)

# if(MSVC)
#     add_compile_options(/wd4273 /wd4996 /WX-)
# endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/simpleble/simpleble)

add_executable(IMUBLE src/imu_ble.cpp)

target_link_libraries(IMUBLE PRIVATE simpleble::simpleble)