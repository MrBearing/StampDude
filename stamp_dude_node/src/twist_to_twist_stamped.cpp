#include "stamp_dude/twist_to_twist_stamped.hpp"

namespace stamp_dude
{
  TwistToTwistStamped::TwistToTwistStamped(const rclcpp::NodeOptions & options) 
    : Stamper<geometry_msgs::msg::Twist, geometry_msgs::msg::TwistStamped>(
        "twist_to_twist_stamped", "twist", "twist_stamped", options)
  {
  }

  void TwistToTwistStamped::convert_message(const geometry_msgs::msg::Twist::SharedPtr input_msg, 
                                           geometry_msgs::msg::TwistStamped & output_msg)
  {
    output_msg.twist = *input_msg;
  }
} // namespace stamp_dude
