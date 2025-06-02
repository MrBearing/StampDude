cmake_minimum_required(VERSION 3.8)
project(stamp_dude)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Define all stamper targets
set(STAMPER_TARGETS
  twist_to_twist_stamped
  point_to_point_stamped
  pose_to_pose_stamped
  quaternion_to_quaternion_stamped
)

# Create libraries for each stamper
foreach(TARGET ${STAMPER_TARGETS})
  set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
  ament_auto_add_library(${MY_LIB_NAME} SHARED src/${TARGET}.cpp)
  target_link_libraries(${MY_LIB_NAME})
endforeach()

# Register components
rclcpp_components_register_node(
  ${PROJECT_NAME}_twist_to_twist_stamped
  PLUGIN "${PROJECT_NAME}::TwistToTwistStamped"
  EXECUTABLE twist_to_twist_stamped_exec)

rclcpp_components_register_node(
  ${PROJECT_NAME}_point_to_point_stamped
  PLUGIN "${PROJECT_NAME}::PointToPointStamped"
  EXECUTABLE point_to_point_stamped_exec)

rclcpp_components_register_node(
  ${PROJECT_NAME}_pose_to_pose_stamped
  PLUGIN "${PROJECT_NAME}::PoseToPoseStamped"
  EXECUTABLE pose_to_pose_stamped_exec)

rclcpp_components_register_node(
  ${PROJECT_NAME}_quaternion_to_quaternion_stamped
  PLUGIN "${PROJECT_NAME}::QuaternionToQuaternionStamped"
  EXECUTABLE quaternion_to_quaternion_stamped_exec)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_twist_to_twist_stamped test/test_twist_to_twist_stamped.cpp)
  target_link_libraries(test_twist_to_twist_stamped ${PROJECT_NAME}_twist_to_twist_stamped)
  ament_target_dependencies(test_twist_to_twist_stamped geometry_msgs)
  
  ament_add_gtest(test_point_to_point_stamped test/test_point_to_point_stamped.cpp)
  target_link_libraries(test_point_to_point_stamped ${PROJECT_NAME}_point_to_point_stamped)
  
  ament_add_gtest(test_pose_to_pose_stamped test/test_pose_to_pose_stamped.cpp)
  target_link_libraries(test_pose_to_pose_stamped ${PROJECT_NAME}_pose_to_pose_stamped)
  
  ament_add_gtest(test_quaternion_to_quaternion_stamped test/test_quaternion_to_quaternion_stamped.cpp)
  target_link_libraries(test_quaternion_to_quaternion_stamped ${PROJECT_NAME}_quaternion_to_quaternion_stamped)
  
  ament_add_gtest(test_scenarios test/test_scenarios.cpp)
  target_link_libraries(test_scenarios ${PROJECT_NAME}_twist_to_twist_stamped)
endif()

ament_auto_package()
