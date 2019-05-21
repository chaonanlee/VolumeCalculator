cmake_minimum_required(VERSION 3.10)
project(demo)

find_package(PCL 1.7 REQUIRED)
set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR /usr/local/opencv4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

set(OPENNI_DIR "/home/louis/drivers/2-Linux/OpenNI-Linux-x64-2.3.0.43")
include_directories( ${OPENNI_DIR}/Include )
link_directories( ${OPENNI_DIR}/Redist )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})


add_executable(demo main.cpp Viewer.cpp Viewer.h OniSampleUtilities.h Depth_Gmm.cpp Depth_Gmm.h)
target_link_libraries(demo libOpenNI2.so)
target_link_libraries (demo ${PCL_LIBRARIES})
target_link_libraries(demo ${OpenCV_LIBS})
target_link_libraries(demo ${GLUT_LIBRARY} ${OPENGL_LIBRARY})