cmake_minimum_required(VERSION 3.0.2)
project(visual_slam)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  eigen_conversions  
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

catkin_package()

include_directories(include  ${catkin_INCLUDE_DIRS})

### Targets ###
add_library(visual_slam
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

add_executable(test_node Executables/test_node_new.cpp)
target_link_libraries(test_node visual_slam ${catkin_LIBRARIES}
g2o_stuff g2o_core g2o_types_slam3d g2o_solver_eigen  )

