cmake_minimum_required(VERSION 2.8.3)
project(iri_poseslam)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS message_generation iri_base_algorithm geometry_msgs sensor_msgs iri_laser_icp pcl_ros laser_geometry)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
find_package(Eigen REQUIRED)
find_package(poseSLAM REQUIRED)

# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Trajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetLink.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/Poseslam.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
# INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
  CATKIN_DEPENDS iri_base_algorithm geometry_msgs sensor_msgs iri_laser_icp laser_geometry pcl_ros
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
  DEPENDS poseSLAM Eigen
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${EIGEN_INCLUDE_DIR}) # include this before catkin to be sure it finds the local eigen version
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${poseSLAM_INCLUDE_DIRS})


## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME}                 src/poseslam_alg.cpp src/poseslam_alg_node.cpp)
add_executable(odom_2_odomrel                  src/odom_2_odomrel_alg.cpp src/odom_2_odomrel_alg_node.cpp)
add_executable(sensors_2_link                  src/sensors_2_link_alg.cpp src/sensors_2_link_alg_node.cpp)
add_executable(trajectory_scans_2_occgrid      src/trajectory_scans_2_occgrid_alg.cpp src/trajectory_scans_2_occgrid_alg_node.cpp)
add_executable(trajectory_scans_2_pointcloud   src/trajectory_scans_2_pointcloud_alg.cpp src/trajectory_scans_2_pointcloud_alg_node.cpp)
add_executable(trajectory_3Dscans_2_pointcloud src/trajectory_3Dscans_2_pointcloud_alg.cpp src/trajectory_3Dscans_2_pointcloud_alg_node.cpp)
add_executable(trajectory_2_markers            src/trajectory_2_markers_alg.cpp src/trajectory_2_markers_alg_node.cpp)
add_executable(trajectory_broadcaster          src/trajectory_broadcaster_alg.cpp src/trajectory_broadcaster_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${EIGEN_LIBRARY} ${poseSLAM_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(odom_2_odomrel ${EIGEN_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(sensors_2_link ${EIGEN_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(trajectory_scans_2_occgrid ${EIGEN_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(trajectory_scans_2_pointcloud ${EIGEN_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(trajectory_3Dscans_2_pointcloud ${EIGEN_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(trajectory_2_markers ${EIGEN_LIBRARY} ${catkin_LIBRARIES} )
target_link_libraries(trajectory_broadcaster ${EIGEN_LIBRARY} ${catkin_LIBRARIES} )


# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
 add_dependencies(${PROJECT_NAME} iri_laser_icp_generate_messages_cpp)
 add_dependencies(${PROJECT_NAME} iri_poseslam_generate_messages_cpp)
 add_dependencies(sensors_2_link iri_laser_icp_generate_messages_cpp)
 add_dependencies(sensors_2_link iri_poseslam_generate_messages_cpp)
 add_dependencies(trajectory_scans_2_occgrid iri_poseslam_generate_messages_cpp)
 add_dependencies(trajectory_scans_2_pointcloud iri_poseslam_generate_messages_cpp)
 add_dependencies(trajectory_3Dscans_2_pointcloud iri_poseslam_generate_messages_cpp)
 add_dependencies(trajectory_2_markers iri_poseslam_generate_messages_cpp)
 add_dependencies(trajectory_broadcaster iri_poseslam_generate_messages_cpp)

# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

