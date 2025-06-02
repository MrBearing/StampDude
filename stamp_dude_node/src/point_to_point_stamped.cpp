#include "stamp_dude/point_to_point_stamped.hpp"

namespace stamp_dude
{
  PointToPointStamped::PointToPointStamped(const rclcpp::NodeOptions & options) 
    : Stamper<geometry_msgs::msg::Point, geometry_msgs::msg::PointStamped>(
        "point_to_point_stamped", "point", "point_stamped", options)
  {
  }

  void PointToPointStamped::convert_message(const geometry_msgs::msg::Point::SharedPtr input_msg, 
                                           geometry_msgs::msg::PointStamped & output_msg)
  {
    output_msg.point = *input_msg;
  }
} // namespace stamp_dude
