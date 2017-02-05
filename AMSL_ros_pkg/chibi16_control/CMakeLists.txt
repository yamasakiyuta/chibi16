cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(chibi16_control src/chibi16_control.cpp)
#rosbuild_add_executable(chibi14_control_qr src/chibi14_control_qr.cpp)
#rosbuild_add_executable(chibi14_control_cl src/chibi14_control_cl.cpp)
#rosbuild_add_executable(roomba_control src/roomba_control.cpp)
#rosbuild_add_executable(roomba_node_manager src/roomba_node_manager.cpp)
#rosbuild_add_executable(node_decider src/node_decider.cpp)
#rosbuild_add_executable(waypoint_saver src/waypoint_saver.cpp)
#target_link_libraries(example ${PROJECT_NAME})
