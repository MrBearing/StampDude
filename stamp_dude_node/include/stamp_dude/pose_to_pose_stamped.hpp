#pragma once

#include "stamp_dude/stamper.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace stamp_dude
{
  class PoseToPoseStamped : public Stamper<geometry_msgs::msg::Pose, geometry_msgs::msg::PoseStamped>
  {
  public:
    explicit PoseToPoseStamped(const rclcpp::NodeOptions & options);

  protected:
    void convert_message(const geometry_msgs::msg::Pose::SharedPtr input_msg, 
                        geometry_msgs::msg::PoseStamped & output_msg) override;
  };
} // namespace stamp_dude

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::PoseToPoseStamped)
