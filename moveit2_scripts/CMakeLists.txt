cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pick_and_place_node src/pick_and_place.cpp)
target_include_directories(pick_and_place_node PUBLIC include)
ament_target_dependencies(pick_and_place_node ament_cmake rclcpp rclcpp_action moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_msgs)

add_executable(pick_and_place_perception_node src/pick_and_place_perception.cpp)
target_include_directories(pick_and_place_perception_node PUBLIC include)
ament_target_dependencies(pick_and_place_perception_node ament_cmake rclcpp rclcpp_action moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_msgs custom_msgs)

add_executable(pick_and_place_perception_real_node src/pick_and_place_perception_real.cpp)
target_include_directories(pick_and_place_perception_real_node PUBLIC include)
ament_target_dependencies(pick_and_place_perception_real_node ament_cmake rclcpp rclcpp_action moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_msgs custom_msgs)


install(TARGETS 
	pick_and_place_node
	pick_and_place_perception_node
	pick_and_place_perception_real_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
