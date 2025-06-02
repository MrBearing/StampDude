#include "stamp_dude/quaternion_to_quaternion_stamped.hpp"

namespace stamp_dude
{
  QuaternionToQuaternionStamped::QuaternionToQuaternionStamped(const rclcpp::NodeOptions & options) 
    : Stamper<geometry_msgs::msg::Quaternion, geometry_msgs::msg::QuaternionStamped>(
        "quaternion_to_quaternion_stamped", "quaternion", "quaternion_stamped", options)
  {
  }

  void QuaternionToQuaternionStamped::convert_message(const geometry_msgs::msg::Quaternion::SharedPtr input_msg, 
                                                     geometry_msgs::msg::QuaternionStamped & output_msg)
  {
    output_msg.quaternion = *input_msg;
  }
} // namespace stamp_dude
