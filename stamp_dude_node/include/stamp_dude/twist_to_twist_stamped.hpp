#pragma once

#include "stamp_dude/stamper.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace stamp_dude
{
  class TwistToTwistStamped : public Stamper<geometry_msgs::msg::Twist, geometry_msgs::msg::TwistStamped>
  {
  public:
    explicit TwistToTwistStamped(const rclcpp::NodeOptions & options);

  protected:
    void convert_message(const geometry_msgs::msg::Twist::SharedPtr input_msg, 
                        geometry_msgs::msg::TwistStamped & output_msg) override;
  };
} // namespace stamp_dude

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::TwistToTwistStamped)
