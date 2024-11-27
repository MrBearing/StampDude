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

namespace stamp_dude_node {

template <typename UnstampedT, typename StampedT, const char* var_name>
class StampConverter : public rclcpp::Node {
public:
  explicit StampConverter(const rclcpp::NodeOptions & options)
  : Node("stamp_converter", options)
  {
    auto callback = [this](const typename UnstampedT::SharedPtr msg) {
      StampedT stamped_msg;
      stamped_msg.header.stamp = this->now();
      stamped_msg.header.frame_id = msg->header.frame_id;

      // ユーザー指定の変数名で値を設定
      stamped_msg.var_name = msg->data; // 例: stringの場合

      // Publish the stamped message (実際のトピック名に置き換える)
      stamped_pub_->publish(stamped_msg);
    };
    unstamped_sub_ = this->create_subscription<UnstampedT>("input", 10, callback);
    stamped_pub_ = this->create_publisher<StampedT>("output", 10);

  }

private:
  typename rclcpp::Subscription<UnstampedT>::SharedPtr unstamped_sub_;
  typename rclcpp::Publisher<StampedT>::SharedPtr stamped_pub_;

};

using AccelToAccelStamped = StampConverter<geometry_msgs::msg::Accel, "accel">;
using AccelWithCovarianceToAccelWithCovarianceStamped = StampConverter<geometry_msgs::msg::AccelWithCovariance, "accel_with_covariance">;
using InertiaToInertiaStamped = StampConverter<geometry_msgs::msg::Inertia, "inertia">;
using PointToPointStamped = StampConverter<geometry_msgs::msg::Point, "point">;
using PolygonToPolygonStamped = StampConverter<geometry_msgs::msg::Polygon, "polygon">;
using PoseToPoseStamped = StampConverter<geometry_msgs::msg::Pose, "pose">;
using PoseWithCovarianceToPoseWithCovarianceStamped = StampConverter<geometry_msgs::msg::PoseWithCovariance, "pose_with_covariance">;
using QuaternionToQuaternionStamped = StampConverter<geometry_msgs::msg::Quaternion, "quaternion">;
using TransformToTransformStamped = StampConverter<geometry_msgs::msg::Transform, "transform">;
using TwistToTwistStamped = StampConverter<geometry_msgs::msg::Twist, "twist">;
using TwistWithCovarianceToTwistWithCovarianceStamped = StampConverter<geometry_msgs::msg::TwistWithCovariance, "twist_with_covariance">;
using Vector3ToVector3Stamped = StampConverter<geometry_msgs::msg::Vector3, "vector3">;
using WrenchToWrenchStamped = StampConverter<geometry_msgs::msg::Wrench, "wrench">;

}  // namespace stamp_dude_node

