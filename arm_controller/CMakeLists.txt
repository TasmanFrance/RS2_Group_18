cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


###############################################################################


#Declare C++ executable
add_executable(ArmController src/controller_main.cpp src/arm_controller.cpp)
target_include_directories(ArmController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
	ArmController
	geometry_msgs
	"rclcpp"
	"moveit_ros_planning_interface")

# Link with MoveIt2 libraries
target_link_libraries(ArmController ${MOVEIT_CORE_LIBRARIES})


###############################################################################


#Declare C++ executable
add_executable(ArmInterface src/interface_main.cpp src/arm_interface.cpp)
target_include_directories(ArmInterface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
	ArmInterface
	geometry_msgs
	visualization_msgs
	tf2_ros
  	tf2_geometry_msgs
	"rclcpp")


###############################################################################


install(TARGETS
    ArmController
    ArmInterface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME})


###############################################################################


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

ament_package()

