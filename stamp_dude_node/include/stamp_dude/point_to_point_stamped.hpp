#pragma once

#include "stamp_dude/stamper.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace stamp_dude
{
  class PointToPointStamped : public Stamper<geometry_msgs::msg::Point, geometry_msgs::msg::PointStamped>
  {
  public:
    explicit PointToPointStamped(const rclcpp::NodeOptions & options);

  protected:
    void convert_message(const geometry_msgs::msg::Point::SharedPtr input_msg, 
                        geometry_msgs::msg::PointStamped & output_msg) override;
  };
} // namespace stamp_dude

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::PointToPointStamped)
