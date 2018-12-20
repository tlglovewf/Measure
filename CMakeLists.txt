cmake_minimum_required(VERSION 2.8)
project(Measure)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -msse3 -std=c++11 -pthread     -g -march=native -Wno-deprecated-declarations")

find_package(OpenCV 3.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenMVG REQUIRED)

include_directories(
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${OpenMvg_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}  main.cpp)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
)
