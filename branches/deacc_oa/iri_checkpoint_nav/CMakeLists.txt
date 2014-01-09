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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(checkpoint_marker src/checkpoint_marker.cpp)

rosbuild_add_executable(checkpoint_manager src/checkpoint_manager.cpp)
target_link_libraries(checkpoint_manager checkpoint_marker)

# C++ Examples
rosbuild_add_executable(checkpoint_marker_example src/examples/checkpoint_marker_example.cpp)
target_link_libraries(checkpoint_marker_example checkpoint_marker)

rosbuild_add_executable(checkpoint_nav_goals_cmd src/examples/checkpoint_nav_goals_cmd.cpp)
rosbuild_add_executable(checkpoint_nav_goals_file src/examples/checkpoint_nav_goals_file.cpp)

# C++ Unit Tests
rosbuild_add_executable(checkpoint_marker_test EXCLUDE_FROM_ALL src/test/checkpoint_marker_test.cpp)
target_link_libraries(checkpoint_marker_test checkpoint_marker)
rosbuild_add_gtest_build_flags(checkpoint_marker_test)
rosbuild_add_rostest(test/test_checkpoint_marker.test)

rosbuild_add_executable(checkpoint_manager_test EXCLUDE_FROM_ALL src/test/checkpoint_manager_test.cpp)
target_link_libraries(checkpoint_manager_test checkpoint_marker)
rosbuild_add_gtest_build_flags(checkpoint_manager_test)
rosbuild_add_rostest(test/test_checkpoint_manager.test)

