cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(PROJECT_NAME poseslam)

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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# added to include support for dynamic reconfiguration
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
# end dynamic reconfiguration

FIND_PACKAGE(iriutils REQUIRED)
FIND_PACKAGE(poseSLAM REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

INCLUDE_DIRECTORIES(${poseSLAM_INCLUDE_DIR} ${iriutils_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR} ./include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/poseslam_alg.cpp src/poseslam_alg_node.cpp)
rosbuild_add_executable(scans_2_odom src/scans_2_odom_alg.cpp src/scans_2_odom_alg_node.cpp)
rosbuild_add_executable(sensors_2_link src/sensors_2_link_alg.cpp src/sensors_2_link_alg_node.cpp)
rosbuild_add_executable(trajectory_scans_2_pointcloud src/trajectory_scans_2_pointcloud_alg.cpp src/trajectory_scans_2_pointcloud_alg_node.cpp)
rosbuild_add_executable(trajectory_3Dscans_2_pointcloud src/trajectory_3Dscans_2_pointcloud_alg.cpp src/trajectory_3Dscans_2_pointcloud_alg_node.cpp)
rosbuild_add_executable(trajectory_2_markers src/trajectory_2_markers_alg.cpp src/trajectory_2_markers_alg_node.cpp)
rosbuild_add_executable(trajectory_broadcaster src/trajectory_broadcaster_alg.cpp src/trajectory_broadcaster_alg_node.cpp)

target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY} ${poseSLAM_LIBRARY} ${EIGEN_LIBRARY})
target_link_libraries(scans_2_odom ${iriutils_LIBRARY})
target_link_libraries(sensors_2_link ${iriutils_LIBRARY})
target_link_libraries(trajectory_scans_2_pointcloud ${iriutils_LIBRARY})
target_link_libraries(trajectory_3Dscans_2_pointcloud ${iriutils_LIBRARY})
target_link_libraries(trajectory_2_markers ${iriutils_LIBRARY})
target_link_libraries(trajectory_broadcaster ${iriutils_LIBRARY})