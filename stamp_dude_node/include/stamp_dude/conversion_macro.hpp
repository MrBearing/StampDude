#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>

#define DEFINE_CONVERSION_NODE(Namespace, UnstampedMsg, StampedMsg, FieldName)   \
  class UnstampedMsg##To##StampedMsg : public rclcpp::Node {                     \
  public:                                                                        \
    explicit UnstampedMsg##To##StampedMsg(const rclcpp::NodeOptions &options)    \
        : rclcpp::Node(#UnstampedMsg "To" #StampedMsg "Node", options) {         \
      sub_ = this->create_subscription<Namespace::UnstampedMsg>(                 \
          "input", 10,                                                           \
          [this](const typename Namespace::UnstampedMsg::SharedPtr msg) {        \
            auto stamped_msg = std::make_shared<Namespace::StampedMsg>();        \
            stamped_msg->header.stamp = this->get_clock()->now();                \
            stamped_msg->FieldName = *msg;                                       \
            pub_->publish(*stamped_msg);                                         \
          });                                                                    \
      pub_ = this->create_publisher<Namespace::StampedMsg>("output", 10);        \
    }                                                                            \
                                                                                 \
  private:                                                                       \
    rclcpp::Subscription<Namespace::UnstampedMsg>::SharedPtr sub_;               \
    rclcpp::Publisher<Namespace::StampedMsg>::SharedPtr pub_;                    \
  };                                                                             \
  RCLCPP_COMPONENTS_REGISTER_NODE(UnstampedMsg##To##StampedMsg)
