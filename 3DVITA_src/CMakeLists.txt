cmake_minimum_required (VERSION 2.8)
project (3DVITA)


# PCL
find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# OpenAL
FIND_PATH(OPENAL_INCLUDE_DIR al.h
  PATHS
  $ENV{OPENAL-SOFT_DIR}
  NO_DEFAULT_PATH
  PATH_SUFFIXES include/AL include/OpenAL include
)

FIND_LIBRARY(OPENAL_LIBRARY 
  NAMES OpenAL al openal OpenAL32
  PATHS
  $ENV{OPENAL-SOFT_DIR}
  NO_DEFAULT_PATH
    PATH_SUFFIXES include/build/Release lib64 lib libs64 libs libs/Win32 libs/Win64
)

include_directories(${OPENAL_INCLUDE_DIR})
link_directories(${OPENAL_LIBRARY})

# STK

# Add PCL


# Separate Executables
add_executable(Grabber grabberMain.cpp)
add_executable(synthesisDemo synthesisMain.cpp)
add_executable(segmentationDemo segmentationMain.cpp)
add_executable(audioDemo audioMain.cpp)


# Grabber Libs
target_link_libraries(Grabber ${PCL_LIBRARIES})

# Synthesis Libs
target_link_libraries(synthesisDemo ${PCL_LIBRARIES})

# Segmentation Libs
target_link_libraries(segmentationDemo ${PCL_LIBRARIES})

# Audio Libs
target_link_libraries(audioDemo ${OPENAL_LIBRARY})