cmake_minimum_required(VERSION 3.8)
project(uav_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable (waypoint_flight_test is the primary C++ flight controller)
add_executable(waypoint_flight_test src/waypoint_flight_test.cpp src/px4_communicator.cpp src/velocity_apf_controller.cpp src/visual_servo_controller.cpp)
add_executable(squre_flight_test src/squre_flight_test.cpp src/px4_communicator.cpp src/velocity_apf_controller.cpp src/visual_servo_controller.cpp)
add_executable(obstacle_course_mission src/obstacle_course_mission.cpp src/px4_communicator.cpp src/velocity_apf_controller.cpp)

# Include directories
target_include_directories(waypoint_flight_test PRIVATE include)
target_include_directories(squre_flight_test PRIVATE include)
target_include_directories(obstacle_course_mission PRIVATE include)

# Target dependencies
ament_target_dependencies(waypoint_flight_test
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_index_cpp
  message_filters
)

ament_target_dependencies(squre_flight_test
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_index_cpp
  message_filters
)

ament_target_dependencies(obstacle_course_mission
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_index_cpp
)

# Link yaml-cpp
target_link_libraries(waypoint_flight_test yaml-cpp)
target_link_libraries(squre_flight_test yaml-cpp)
target_link_libraries(obstacle_course_mission yaml-cpp)

# Install executable
install(TARGETS
  waypoint_flight_test
  squre_flight_test
  obstacle_course_mission
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/servo_control_node.py
  scripts/takeoff_control.py
  scripts/circle_landing_target_node.py
  scripts/qr_target_report_node.py
  scripts/air_ground_coop_node.py
  scripts/laser_scan_to_obstacle_distance.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # find_package(ament_cmake_gtest REQUIRED)
  
  # Unit tests for px4_comm_types.hpp - PASSED ✓
  # Generated and validated by AI (GitHub Copilot)
  # Tests cover: construction, coordinate conversion, matrix properties, edge cases
  # 
  # To enable tests, uncomment the following lines:
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_px4_comm_types_quick test/test_px4_comm_types_quick.cpp)
  # target_include_directories(test_px4_comm_types_quick PRIVATE include /usr/include/eigen3)
  # ament_target_dependencies(test_px4_comm_types_quick rclcpp)
  # 
  # ament_add_gtest(test_px4_comm_types test/test_px4_comm_types.cpp)
  # target_include_directories(test_px4_comm_types PRIVATE include /usr/include/eigen3)
  # ament_target_dependencies(test_px4_comm_types rclcpp)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
