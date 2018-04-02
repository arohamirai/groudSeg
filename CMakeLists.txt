# This is demo on how to link libpointmatcher to an
# external project.

cmake_minimum_required (VERSION 2.8)
project(groudSeg)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules  /usr/local/*)
set(OpenCV_DIR /usr/local/opencv3.4.1/share/OpenCV)

find_package(VTK 6.2 REQUIRED)
find_package(OpenCV 3.4.1 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system )
find_package(PCL REQUIRED)
include(${VTK_USE_FILE})

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(
    include
    #${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ./src/groudSeg.cpp)

target_link_libraries(${PROJECT_NAME}
    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)
