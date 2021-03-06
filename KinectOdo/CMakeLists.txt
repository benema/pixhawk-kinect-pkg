cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(KinectOdo /home/benema/ros_tutorials/Kinect-Pixhawk/pixhawk-kinect-pkg/toro/trunk/treeoptimizer3.cpp)
#rosbuild_add_library(KinectOdo /home/benema/ros_tutorials/Kinect-Pixhawk/pixhawk-kinect-pkg/toro/trunk/posegraph3.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(KinectOdo TORO)
SET(
  CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
)
include(cmake/FindGLIB2.cmake)

FIND_PACKAGE(GLIB2 REQUIRED)

INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
)

rosbuild_add_executable(KinectOdo src/main.cpp src/extractFeature.h src/extractFeature.cpp ../toro/trunk/treeoptimizer3.cpp ../toro/trunk/posegraph3.cpp ../toro/trunk/treeoptimizer3_iteration.cpp)
