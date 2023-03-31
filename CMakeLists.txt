cmake_minimum_required(VERSION 3.5)

project(borinot_jump)

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
find_package(ament_cmake REQUIRED)
find_package(eagle_ros2_interfaces REQUIRED)
find_package(eagle_ros2_tf REQUIRED COMPONENTS eagle_ros2_tf_tools)
find_package(eagle_ros2_control REQUIRED COMPONENTS eagle_ros2_control_tools)
find_package(Eigen3 REQUIRED)
find_package(hidro_ros2_utils REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)

# ---- LIBRARY: BORINOT_JUMP_TOOLS ----
list(APPEND BORINOT_JUMP_TOOLS_SRC src/jump_controller.cpp)

add_library(${PROJECT_NAME}_tools SHARED ${BORINOT_JUMP_TOOLS_SRC})

target_include_directories(
  ${PROJECT_NAME}_tools
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_tools rclcpp)
ament_target_dependencies(${PROJECT_NAME}_tools Eigen3)
ament_target_dependencies(${PROJECT_NAME}_tools hidro_ros2_utils)
ament_target_dependencies(${PROJECT_NAME}_tools px4_msgs)
ament_target_dependencies(${PROJECT_NAME}_tools px4_ros_com)
ament_target_dependencies(${PROJECT_NAME}_tools eagle_ros2_interfaces)
ament_target_dependencies(${PROJECT_NAME}_tools std_srvs)

target_link_libraries(${PROJECT_NAME}_tools eagle_ros2_tf::eagle_ros2_tf_tools)
target_link_libraries(${PROJECT_NAME}_tools eagle_ros2_control::eagle_ros2_control_tools)

# target_link_libraries(${PROJECT_NAME}_tools hidro_ros2_utils::hidro_ros2_utils_lib)
list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${PROJECT_NAME}_tools)

# ---- NODE: JUMP NODE ----
list(APPEND JUMP_CTRL_SRC src/nodes/jump_ctrl.cpp)

set(JUMP_CTRL_EXEC_NAME jump_ctrl)

add_executable(${JUMP_CTRL_EXEC_NAME} ${JUMP_CTRL_SRC})
target_link_libraries(${JUMP_CTRL_EXEC_NAME} ${PROJECT_NAME}_tools)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${JUMP_CTRL_EXEC_NAME})

# ---- INSTALL TARGETS ----
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(Eigen3)
ament_export_dependencies(hidro_ros2_utils)
ament_export_dependencies(px4_msgs)
ament_export_dependencies(px4_ros_com)
ament_export_dependencies(eagle_ros2_interfaces)
ament_export_dependencies(eagle_ros2_tf)
ament_export_dependencies(eagle_ros2_control)
ament_export_dependencies(std_srvs)

# Install folders
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install targets
install(
  TARGETS ${${PROJECT_NAME}_INSTALL_TARGETS}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

ament_package()
