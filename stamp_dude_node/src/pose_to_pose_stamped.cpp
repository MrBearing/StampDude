#include "stamp_dude/pose_to_pose_stamped.hpp"

namespace stamp_dude
{
  PoseToPoseStamped::PoseToPoseStamped(const rclcpp::NodeOptions & options) 
    : Stamper<geometry_msgs::msg::Pose, geometry_msgs::msg::PoseStamped>(
        "pose_to_pose_stamped", "pose", "pose_stamped", options)
  {
  }

  void PoseToPoseStamped::convert_message(const geometry_msgs::msg::Pose::SharedPtr input_msg, 
                                         geometry_msgs::msg::PoseStamped & output_msg)
  {
    output_msg.pose = *input_msg;
  }
} // namespace stamp_dude
