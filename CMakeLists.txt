cmake_minimum_required(VERSION 3.5)
project(ars_40X)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
if ($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED
      COMPONENTS
      roscpp
      message_generation
      )
else ()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
endif ()

find_package(costmap_converter REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(socket_can REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

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
endif ()

include_directories(include)
include_directories(${socket_can_INCLUDE_DIRS})

add_library(ars_40X_msg
    src/cluster_list.cpp
    src/motion_input_signals.cpp
    src/object_list.cpp
    src/radar_cfg.cpp
    src/radar_state.cpp
    )

add_library(ars_40X_can
    src/ars_40X_can.cpp
    )

target_link_libraries(ars_40X_can ars_40X_msg ${socket_can_LIBRARIES})

add_executable(ars_40X_ros
    src/ros/ars_40X_ros.cpp
    src/ros/cluster_list_ros.cpp
    src/ros/motion_input_signals_ros.cpp
    src/ros/object_list_ros.cpp
    src/ros/radar_cfg_ros.cpp
    src/ros/radar_state_ros.cpp
    )

add_dependencies(ars_40X_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ars_40X_ros ${catkin_LIBRARIES})

add_executable(ars_40X_rviz
    src/ros/ars_40X_rviz.cpp
    )
add_dependencies(ars_40X_rviz ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ars_40X_rviz ${catkin_LIBRARIES})

add_executable(ars_40X_obstacle_array
    src/ros/ars_40X_obstacle_array.cpp
    )
add_dependencies(ars_40X_obstacle_array ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ars_40X_obstacle_array ${catkin_LIBRARIES})

if ($ENV{ROS_VERSION} EQUAL 1)
  target_link_libraries(ars_40X_ros ars_40X_can ${catkin_LIBRARIES})
endif ()

install(
    TARGETS
    ars_40X_can
    ars_40X_msg
    ars_40X_ros
    ars_40X_rviz
    ars_40X_obstacle_array
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if ($ENV{ROS_VERSION} EQUAL 2)
  ament_package()
endif ()
