cmake_minimum_required(VERSION 3.5)
project(observe)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)

find_package(trajectory_msgs REQUIRED)

find_package(ros2_knowledge_graph_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()  
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    geometry_msgs
    gazebo_msgs
    ament_index_cpp
    ros2_knowledge_graph
    trajectory_msgs
    ros2_knowledge_graph_msgs
    
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/observe/Observator.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(observe_object_node src/observe_object_node.cpp)
ament_target_dependencies(observe_object_node ${dependencies})
target_link_libraries(observe_object_node ${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  observe_object_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
