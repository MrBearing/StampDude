#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

namespace stamp_dude
{
  template<typename UnstampedMsg, typename StampedMsg>
  class Stamper : public rclcpp::Node
  {
  public:
    explicit Stamper(
      const std::string & node_name,
      const std::string & input_topic,
      const std::string & output_topic,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node(node_name, options)
    {
      // Declare the frame_id parameter once during initialization
      frame_id_ = this->declare_parameter("frame_id", "base_link");
      
      publisher_ = this->create_publisher<StampedMsg>(output_topic, 10);
      subscription_ = this->create_subscription<UnstampedMsg>(
        input_topic, 10, 
        std::bind(&Stamper::message_callback, this, std::placeholders::_1));
      
      RCLCPP_INFO(this->get_logger(), "Started %s: %s -> %s", 
                  node_name.c_str(), input_topic.c_str(), output_topic.c_str());
    }

  protected:
    virtual void convert_message(const typename UnstampedMsg::SharedPtr input_msg, StampedMsg & output_msg) = 0;

  private:
    void message_callback(const typename UnstampedMsg::SharedPtr msg)
    {
      auto stamped_msg = StampedMsg();
      stamped_msg.header.stamp = this->get_clock()->now();
      stamped_msg.header.frame_id = frame_id_;
      
      convert_message(msg, stamped_msg);
      publisher_->publish(stamped_msg);
    }

    typename rclcpp::Publisher<StampedMsg>::SharedPtr publisher_;
    typename rclcpp::Subscription<UnstampedMsg>::SharedPtr subscription_;
    std::string frame_id_;
  };
} // namespace stamp_dude
