################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_motion)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  open_manipulator_msgs
  robotis_manipulator
  open_manipulator_6dof_libs
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_message_files(
   FILES
   MotionState.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include  
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs open_manipulator_msgs robotis_manipulator open_manipulator_6dof_libs message_runtime
  
)

################################################################################
# Build
################################################################################
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_motion src/open_manipulator_motion.cpp)
add_dependencies(open_manipulator_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_motion ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_motion
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################

