cmake_minimum_required(VERSION 2.8.3)
project(cwru_pcl_utils)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  std_msgs
  tf
  cwru_srv
)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES cwru_pcl_utils
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions sensor_msgs std_msgs tf cwru_srv
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Uncomment this when cwru_pcl_utils is fixed
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Werror -Wno-unused-parameter")

add_library(cwru_pcl_utils
  src/cwru_pcl_utils.cpp
)

target_link_libraries(cwru_pcl_utils 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(cwru_pcl_utils_example_main
  src/cwru_pcl_utils_example_main.cpp
)

add_dependencies(cwru_pcl_utils_example_main cwru_pcl_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cwru_pcl_utils_example_main
  ${catkin_LIBRARIES}
  cwru_pcl_utils
)
