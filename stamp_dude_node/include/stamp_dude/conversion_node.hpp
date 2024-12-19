#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_instance.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <chrono>

#include "conversion_macro.hpp"

namespace stamp_dude {

// Pose -> PoseStamped ノードの生成
DEFINE_CONVERSION_NODE(geometry_msgs::msg, Pose, PoseStamped, pose);// stamp_dude::PoseToPoseStamped

// Accel -> AccelStamped ノードの生成
DEFINE_CONVERSION_NODE(geometry_msgs::msg, Accel, AccelStamped, accel); // stamp_dude::AccelToAccelStamped

}
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::AccelToAccelStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::AccelWithCovarianceToAccelWithCovarianceStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::InertiaToInertiaStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::PointToPointStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::PolygonToPolygonStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::PoseToPoseStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::PoseWithCovarianceToPoseWithCovarianceStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::QuaternionToQuaternionStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::TransformToTransformStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::TwistToTwistStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::TwistWithCovarianceToTwistWithCovarianceStamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::Vector3ToVector3Stamped)
// RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::WrenchToWrenchStamped)