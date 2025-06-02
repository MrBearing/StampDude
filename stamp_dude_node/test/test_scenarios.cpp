#include "stamp_dude/twist_to_twist_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

TEST(TwistToTwistStampedScenario, MessageConversionTest)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<stamp_dude::TwistToTwistStamped>(options);
  
  // Create a publisher to send messages to the node
  auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("twist", 10);
  
  // Create a subscription to receive the stamped output
  geometry_msgs::msg::TwistStamped::SharedPtr received_msg;
  auto twist_stamped_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "twist_stamped", 10,
    [&received_msg](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      received_msg = msg;
    });

  // Create and publish a test message
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 1.0;
  twist_msg.angular.z = 0.5;
  
  // Spin to process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  twist_pub->publish(twist_msg);
  
  // Give some time for the message to be processed
  auto start_time = std::chrono::steady_clock::now();
  while (!received_msg && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  
  // Verify the conversion
  ASSERT_NE(nullptr, received_msg);
  EXPECT_EQ(twist_msg.linear.x, received_msg->twist.linear.x);
  EXPECT_EQ(twist_msg.angular.z, received_msg->twist.angular.z);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
