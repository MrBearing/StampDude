cmake_minimum_required(VERSION 3.8)
project(stamp_dude)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TARGET twist_to_twist_stamped)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::TwistToTwistStamped"
  EXECUTABLE ${TARGET}_exec)
target_link_libraries(${MY_LIB_NAME})


# ament_auto_add_library(${PROJECT_NAME} SHARED src/stamp_converter.cpp)

# set(TARGET pose_to_pose_stamped)
# set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
# rclcpp_components_register_nodes( 
# ${MY_LIB_NAME}
# PLUGIN "${PROJECT_NAME}::PoseToPoseStamped"
# EXECUTABLE ${TARGET}_exec)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
