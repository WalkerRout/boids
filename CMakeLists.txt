cmake_minimum_required(VERSION 3.15)
project(boids C)

set(CMAKE_C_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCC)
  message(STATUS "gcc detected, adding specific flags")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall -Wextra -Wpedantic -Werror")
else()
  message(STATUS "Detected non-gcc compiler, omitting some flags")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
endif(CMAKE_COMPILER_IS_GNUCC)

find_package(raylib REQUIRED)

file(GLOB SOURCES "src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

find_library(LIBM m)
if (LIBM)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBM})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC raylib)

target_include_directories(${PROJECT_NAME} PUBLIC include)
