cmake_minimum_required (VERSION 2.8)
project (3DVITA)

# PCL - Library to deal with point clouds and to use RGBD sensors with OPENNI
find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV - Computer Vision library for image processing
FIND_PACKAGE ( OpenCV REQUIRED core highgui imgproc)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR} )

# STK - Synthesis ToolKit to generate sound



# Separate Executables
add_executable(Grabber grabberMain.cpp)


# Grabber Libs
target_link_libraries(Grabber ${PCL_LIBRARIES})
target_link_libraries(Grabber ${OpenCV_LIBS} )
