cmake_minimum_required(VERSION 3.15)
project(boids C)

set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -Wpedantic -Werror ${CMAKE_C_FLAGS}")

find_package(raylib REQUIRED)

file(GLOB SOURCES "src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

find_library(LIBM m)
if (LIBM)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBM})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC raylib)

target_include_directories(${PROJECT_NAME} PUBLIC include)
