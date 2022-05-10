cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME realsense_interface)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)


include_directories(
  include
  include/${PROJECT_NAME}
  ${realsense2_INCLUDE_DIR}
)

set(HEADER_HPP_FILES 
  include/${PROJECT_NAME}/rs_t265_interface.hpp  
)

set(SOURCE_CPP_FILES 
  src/rs_t265_interface.cpp
)

#add_executable(test_pose src/test_pose.cpp)
#target_link_libraries(test_pose ${realsense2_LIBRARY})
#ament_target_dependencies(test_pose rclcpp std_msgs realsense2 as2_core as2_msgs)

add_executable(${PROJECT_NAME}_node src/rs_t265_interface_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${realsense2_LIBRARY})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp tf2 tf2_ros tf2_geometry_msgs std_msgs sensor_msgs nav_msgs geometry_msgs std_srvs as2_core as2_msgs)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
