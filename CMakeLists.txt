cmake_minimum_required(VERSION 3.2)
project(ObjectFinder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)
INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
    ${PCL_LIBRARY_DIRS}
)
ADD_DEFINITIONS(
    ${PCL_DEFINITIONS}
)

set(LIBRARIES
    ${OPENCV_LIBS}
    ${OPENCV_LIBRARIES}
)

set(SOURCE_FILES main.cpp)
add_executable(ObjectFinder ${SOURCE_FILES} common.h IPointCloudDivider.h ProjectionDivider.cpp ProjectionDivider.h pointcloudUtilities.h pointcloudUtilities.cpp KinectParameters.h KinectParameters.cpp ImageRebuild.h ImageRebuild.cpp)

target_link_libraries(ObjectFinder
${OpenCV_LIBS}
${PCL_LIBRARIES}
)