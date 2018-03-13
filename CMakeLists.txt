cmake_minimum_required(VERSION 2.8.3)
project(ros_gpg)
#set(CMAKE_BUILD_TYPE Release)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
)
find_package(Eigen3 REQUIRED)
# Eigen library
#include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare")
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS cmake_modules eigen_conversions roscpp
   LIBRARIES ${PROJECT_NAME}_grasp_candidates_generator
   DEPENDS Eigen PCL
)
include_directories(
 include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
# Generate the shared library from the sources
add_library(${PROJECT_NAME}_grasp_candidates_generator src/${PROJECT_NAME}/candidates_generator.cpp)

# Other libraries
add_library(${PROJECT_NAME}_antipodal src/${PROJECT_NAME}/antipodal.cpp)
add_library(${PROJECT_NAME}_cloud_camera src/${PROJECT_NAME}/cloud_camera.cpp)
add_library(${PROJECT_NAME}_config_file src/${PROJECT_NAME}/config_file.cpp)
add_library(${PROJECT_NAME}_eigen_utils src/${PROJECT_NAME}/eigen_utils.cpp)
add_library(${PROJECT_NAME}_finger_hand src/${PROJECT_NAME}/finger_hand.cpp)
add_library(${PROJECT_NAME}_frame_estimator src/${PROJECT_NAME}/frame_estimator.cpp)
add_library(${PROJECT_NAME}_grasp src/${PROJECT_NAME}/grasp.cpp)
add_library(${PROJECT_NAME}_grasp_set src/${PROJECT_NAME}/grasp_set.cpp)
add_library(${PROJECT_NAME}_hand_search src/${PROJECT_NAME}/hand_search.cpp)
add_library(${PROJECT_NAME}_local_frame src/${PROJECT_NAME}/local_frame.cpp)
add_library(${PROJECT_NAME}_plot src/${PROJECT_NAME}/plot.cpp)
add_library(${PROJECT_NAME}_point_list src/${PROJECT_NAME}/point_list.cpp)
# This executable is for testing the shared library
add_executable(${PROJECT_NAME}_generate_candidates src/generate_candidates.cpp)
target_link_libraries(${PROJECT_NAME}_generate_candidates
                      ${PROJECT_NAME}_config_file
                      ${PROJECT_NAME}_grasp_candidates_generator)

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_antipodal
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_cloud_camera
                      ${PROJECT_NAME}_eigen_utils
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_eigen_utils
                      ${EIGEN_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_frame_estimator
                      ${PROJECT_NAME}_cloud_camera
                      ${PROJECT_NAME}_local_frame)

target_link_libraries(${PROJECT_NAME}_grasp
                      ${PROJECT_NAME}_antipodal
                      ${PROJECT_NAME}_finger_hand
                      ${PROJECT_NAME}_local_frame
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_grasp_set
                      ${PROJECT_NAME}_antipodal
                      ${PROJECT_NAME}_grasp
                      ${PROJECT_NAME}_local_frame
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_hand_search
                      ${PROJECT_NAME}_antipodal
                      ${PROJECT_NAME}_cloud_camera
                      ${PROJECT_NAME}_frame_estimator
                      ${PROJECT_NAME}_grasp_set
                      ${PROJECT_NAME}_plot)

target_link_libraries(${PROJECT_NAME}_local_frame
                      ${PCL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_plot
                      ${PROJECT_NAME}_cloud_camera
                      ${PROJECT_NAME}_grasp_set
                      ${PROJECT_NAME}_local_frame)

target_link_libraries(${PROJECT_NAME}_grasp_candidates_generator
                      ${PROJECT_NAME}_hand_search)

target_link_libraries(${PROJECT_NAME}_point_list
                      ${PROJECT_NAME}_eigen_utils)


# Rename executable.
set_target_properties(${PROJECT_NAME}_generate_candidates
                      PROPERTIES OUTPUT_NAME generate_candidates_ros
                      PREFIX "")

#set_target_properties(${PROJECT_NAME}_grasp_candidates_generator
  #                    PROPERTIES OUTPUT_NAME grasp_candidates_generator_ros)

# Set the location for library installation (/usr/lib in this case is not really necessary)
# Use "sudo make install" to apply
#install(TARGETS ${PROJECT_NAME}_grasp_candidates_generator
  #     DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/gpg DESTINATION include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
