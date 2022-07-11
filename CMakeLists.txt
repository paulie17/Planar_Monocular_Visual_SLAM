cmake_minimum_required(VERSION 3.0.2)
project(planar_monocular_slam_thesis)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  eigen_conversions  
  genmsg
  tf
  tf2
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

## Declare ROS messages and services
add_service_files(FILES BARequest.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include  ${catkin_INCLUDE_DIRS})

### Targets ###
add_library(planar_monocular_slam
            include/utils.hpp
            include/g2otypes.hpp  
            src/cameraManager.cpp
            src/Map.cpp 
            src/MapPoint.cpp 
            src/visual_slam_interface.cpp
            src/condensed_optimizer.cpp)
# target_link_libraries(visual_slam
#                       ${OpenCV_LIBRARIES}
#                       ${Boost_LIBRARIES})

add_executable(planar_monocular_slam_node Executables/planar_monocular_slam_node.cpp)
target_link_libraries(planar_monocular_slam_node planar_monocular_slam ${catkin_LIBRARIES}
g2o_stuff g2o_core g2o_types_slam3d g2o_solver_eigen g2o_types_slam2d 
g2o_solver_cholmod cholmod 
# g2o_solver_csparse
# g2o_csparse_extension
# g2o_solver_pcg
)
add_dependencies(planar_monocular_slam_node planar_monocular_slam_thesis_gencpp)

add_executable(tf_to_path Executables/tf_to_path.cpp)
target_link_libraries(tf_to_path ${catkin_LIBRARIES})

add_executable(evaluation_node Executables/evaluation_node.cpp)
target_link_libraries(evaluation_node ${catkin_LIBRARIES})



