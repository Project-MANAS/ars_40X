cmake_minimum_required(VERSION 3.5)
project(continental_radar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
if ($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
endif()

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(socket_can REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)
include_directories(${socket_can_INCLUDE_DIRS})

if ($ENV{ROS_VERSION} EQUAL 1)
  add_message_files(DIRECTORY msg
    FILES
    Cluster.msg
    ClusterList.msg
    Object.msg
    ObjectList.msg
    RadarStatus.msg
  )

  add_service_files(
    FILES
    MaxDistance.srv
    OutputType.srv
    RadarPower.srv
    RCSThreshold.srv
    SensorID.srv
    SortIndex.srv
  )

  generate_messages(DEPENDENCIES std_msgs geometry_msgs)

  catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}_can
      ${PROJECT_NAME}_msg
    CATKIN_DEPENDS
      message_runtime
      roscpp
      socket_can
  )
  include_directories(${catkin_INCLUDE_DIRS})
endif()

add_library(continental_radar_msg
  src/cluster_list.cpp
  src/motion_input_signals.cpp
  src/object_list.cpp
  src/radar_cfg.cpp
  src/radar_state.cpp
)

add_library(continental_radar_can
  src/continental_radar_can.cpp
)

target_link_libraries(continental_radar_can continental_radar_msg ${socket_can_LIBRARIES})

add_executable(continental_radar_ros
  src/ros/continental_radar_ros.cpp
  src/ros/cluster_list_ros.cpp
  src/ros/motion_input_signals_ros.cpp
  src/ros/object_list_ros.cpp
  src/ros/radar_cfg_ros.cpp
  src/ros/radar_state_ros.cpp
)

add_executable(continental_radar_rviz
    src/ros/continental_radar_rviz.cpp
)
target_link_libraries(continental_radar_rviz ${catkin_LIBRARIES})

if ($ENV{ROS_VERSION} EQUAL 1)
  add_dependencies(continental_radar_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
  target_link_libraries(continental_radar_ros continental_radar_can ${catkin_LIBRARIES})
endif()

install(
  TARGETS
    continental_radar_can
    continental_radar_msg
    continental_radar_ros
    continental_radar_rviz
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if ($ENV{ROS_VERSION} EQUAL 2)
  ament_package()
endif()
