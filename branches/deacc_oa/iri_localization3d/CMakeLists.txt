cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE(iriutils REQUIRED)
FIND_PACKAGE(position_3d REQUIRED)
FIND_PACKAGE(faramotics REQUIRED)
FIND_PACKAGE(map_localization REQUIRED)

set(PROJECT_NAME localization3d_alg_node)

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
#rosbuild_gensrv()

# added to include support for dynamic reconfiguration
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
# end dynamic reconfiguration

INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ./include)
INCLUDE_DIRECTORIES(${map_localization_INCLUDE_DIR})
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(/usr/local/include/eigen3)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/localization3d_alg.cpp src/localization3d_alg_node.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${iriutils_LIBRARY})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${position3d_LIBRARY})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${faramotics_LIBRARY})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${map_localization_LIBRARY})
