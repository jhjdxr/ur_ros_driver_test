cmake_minimum_required(VERSION 3.8)
project(ur_script_sender_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

add_executable(send_script src/send_script.cpp)
ament_target_dependencies(send_script rclcpp std_msgs)
install(TARGETS send_script DESTINATION lib/${PROJECT_NAME})

add_executable(send_traj_action src/send_traj_action.cpp)
ament_target_dependencies(send_traj_action
  rclcpp rclcpp_action
  sensor_msgs trajectory_msgs control_msgs
  controller_manager_msgs
  ament_index_cpp
)

# yaml-cpp::yaml-cpp 대신 yaml-cpp 로 링크
target_link_libraries(send_traj_action yaml-cpp)

install(TARGETS send_traj_action DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY paths/
  DESTINATION share/${PROJECT_NAME}/paths
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

ament_package()
