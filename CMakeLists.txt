cmake_minimum_required(VERSION 3.15)
project(boids C)

if(CMAKE_COMPILER_IS_GNUCXX)
  message(STATUS "gcc detected, adding specific flags")
  set(CMAKE_C_FLAGS "-Wall -Wextra -Wpedantic -Werror ${CMAKE_C_FLAGS}")
else()
  message(STATUS "Detected non-gcc compiler, omitting some flags")
  set(CMAKE_C_FLAGS "-Wall ${CMAKE_C_FLAGS}")
endif(CMAKE_COMPILER_IS_GNUCXX)

find_package(raylib REQUIRED)

file(GLOB SOURCES "src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

find_library(LIBM m)
if (LIBM)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBM})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC raylib)

target_include_directories(${PROJECT_NAME} PUBLIC include)
