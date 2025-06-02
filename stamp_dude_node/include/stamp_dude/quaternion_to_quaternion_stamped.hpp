#pragma once

#include "stamp_dude/stamper.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

namespace stamp_dude
{
  class QuaternionToQuaternionStamped : public Stamper<geometry_msgs::msg::Quaternion, geometry_msgs::msg::QuaternionStamped>
  {
  public:
    explicit QuaternionToQuaternionStamped(const rclcpp::NodeOptions & options);

  protected:
    void convert_message(const geometry_msgs::msg::Quaternion::SharedPtr input_msg, 
                        geometry_msgs::msg::QuaternionStamped & output_msg) override;
  };
} // namespace stamp_dude

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::QuaternionToQuaternionStamped)
