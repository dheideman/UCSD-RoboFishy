cmake_minimum_required(VERSION 2.8)
project( OpticalFlowDemo )
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( OpticalFlowDemo OpticalFlowDemo.cpp )
target_link_libraries( OpticalFlowDemo ${OpenCV_LIBS} )
