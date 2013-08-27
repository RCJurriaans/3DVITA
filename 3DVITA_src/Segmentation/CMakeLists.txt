cmake_minimum_required (VERSION 2.8)
project (Segmentation)

# PCL - Library to deal with point clouds and to use RGBD sensors with OPENNI
find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV - Computer Vision library for image processing
FIND_PACKAGE ( OpenCV REQUIRED core highgui imgproc)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR} )

# OpenAL - Sound Library for playback
include(FindOpenAL REQUIRED)

include_directories(${OPENAL_INCLUDE_DIR})
link_directories(${OPENAL_LIBRARY})

# Separate Executables
add_executable(Grabber grabber.cpp grabber.h segmentation.cpp segmentation.h audiogenerator.cpp audiogenerator.h audioObject.cpp audioObject.h)

# Grabber Libs
target_link_libraries(Grabber ${PCL_LIBRARIES})
target_link_libraries(Grabber ${OpenCV_LIBS} )
target_link_libraries(Grabber ${OPENAL_LIBRARY} )