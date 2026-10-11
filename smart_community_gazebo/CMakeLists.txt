cmake_minimum_required(VERSION 3.8)

project(smart_community_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)



# Fire plugin
add_library(fire_plugin SHARED
  src/fire_plugin.cpp
)

ament_target_dependencies(
  fire_plugin
  rclcpp
  std_msgs
)

target_link_libraries(
  fire_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS fire_plugin
  LIBRARY DESTINATION lib
)



ament_package()

# Traffic light simulation-time synchronized visual plugin
add_library(traffic_light_sync_plugin SHARED
  src/traffic_light_sync_plugin.cpp
)

target_link_libraries(traffic_light_sync_plugin
  ${gazebo_LIBRARIES}
)

install(TARGETS traffic_light_sync_plugin
  LIBRARY DESTINATION lib
)
