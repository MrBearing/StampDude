#include "stamp_dude/twist_to_twist_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <chrono>
#include <thread>

class TwistToTwistStampedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    node_ = std::make_shared<stamp_dude::TwistToTwistStamped>(options);
    
    // Create publisher to send messages to the node
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("twist", 10);
    
    // Create subscription to receive stamped messages from the node
    sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "twist_stamped", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_stamped_msg_ = msg;
        message_received_ = true;
      });
    
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }
  
  void TearDown() override
  {
    node_.reset();
    executor_.reset();
  }
  
  void WaitForMessage(std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
  {
    auto start_time = std::chrono::steady_clock::now();
    while (!message_received_ && 
           (std::chrono::steady_clock::now() - start_time) < timeout) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  
  std::shared_ptr<stamp_dude::TwistToTwistStamped> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_stamped_msg_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  bool message_received_ = false;
};

TEST_F(TwistToTwistStampedTest, InitializationTest)
{
  ASSERT_NE(nullptr, node_);
}

TEST_F(TwistToTwistStampedTest, MessageConversionTest)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 1.0;
  twist_msg.linear.y = 2.0;
  twist_msg.linear.z = 3.0;
  twist_msg.angular.x = 0.1;
  twist_msg.angular.y = 0.2;
  twist_msg.angular.z = 0.3;
  
  message_received_ = false;
  pub_->publish(twist_msg);
  
  WaitForMessage();
  
  ASSERT_TRUE(message_received_);
  ASSERT_NE(nullptr, last_stamped_msg_);
  
  // Check frame_id
  EXPECT_EQ("base_link", last_stamped_msg_->header.frame_id);
  
  // Check linear velocity values
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->twist.linear.x);
  EXPECT_DOUBLE_EQ(2.0, last_stamped_msg_->twist.linear.y);
  EXPECT_DOUBLE_EQ(3.0, last_stamped_msg_->twist.linear.z);
  
  // Check angular velocity values
  EXPECT_DOUBLE_EQ(0.1, last_stamped_msg_->twist.angular.x);
  EXPECT_DOUBLE_EQ(0.2, last_stamped_msg_->twist.angular.y);
  EXPECT_DOUBLE_EQ(0.3, last_stamped_msg_->twist.angular.z);
  
  // Check timestamp
  auto stamp_sec = last_stamped_msg_->header.stamp.sec;
  auto stamp_nanosec = last_stamped_msg_->header.stamp.nanosec;
  EXPECT_GT(stamp_sec, 0);
  EXPECT_GE(stamp_nanosec, 0u);
}

TEST_F(TwistToTwistStampedTest, ZeroVelocityTest)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  // All values default to 0.0
  
  message_received_ = false;
  pub_->publish(twist_msg);
  
  WaitForMessage();
  
  ASSERT_TRUE(message_received_);
  ASSERT_NE(nullptr, last_stamped_msg_);
  
  // Check all values are zero
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->twist.linear.x);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->twist.linear.y);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->twist.linear.z);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->twist.angular.x);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->twist.angular.y);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->twist.angular.z);
}

TEST_F(TwistToTwistStampedTest, MultipleMessagesTest)
{
  // First message
  auto twist_msg1 = geometry_msgs::msg::Twist();
  twist_msg1.linear.x = 1.0;
  twist_msg1.angular.z = 0.5;
  
  message_received_ = false;
  pub_->publish(twist_msg1);
  WaitForMessage();
  
  ASSERT_TRUE(message_received_);
  auto first_stamp = last_stamped_msg_->header.stamp;
  
  // Small delay to ensure different timestamp
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  // Second message
  auto twist_msg2 = geometry_msgs::msg::Twist();
  twist_msg2.linear.x = 2.0;
  twist_msg2.angular.z = 1.0;
  
  message_received_ = false;
  pub_->publish(twist_msg2);
  WaitForMessage();
  
  ASSERT_TRUE(message_received_);
  EXPECT_DOUBLE_EQ(2.0, last_stamped_msg_->twist.linear.x);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->twist.angular.z);
  
  // Check that timestamp is different
  auto second_stamp = last_stamped_msg_->header.stamp;
  EXPECT_GT(rclcpp::Time(second_stamp), rclcpp::Time(first_stamp));
}

TEST_F(TwistToTwistStampedTest, NegativeVelocityTest)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = -1.5;
  twist_msg.linear.y = -2.5;
  twist_msg.linear.z = -3.5;
  twist_msg.angular.x = -0.15;
  twist_msg.angular.y = -0.25;
  twist_msg.angular.z = -0.35;
  
  message_received_ = false;
  pub_->publish(twist_msg);
  
  WaitForMessage();
  
  ASSERT_TRUE(message_received_);
  ASSERT_NE(nullptr, last_stamped_msg_);
  
  // Check negative values are preserved
  EXPECT_DOUBLE_EQ(-1.5, last_stamped_msg_->twist.linear.x);
  EXPECT_DOUBLE_EQ(-2.5, last_stamped_msg_->twist.linear.y);
  EXPECT_DOUBLE_EQ(-3.5, last_stamped_msg_->twist.linear.z);
  EXPECT_DOUBLE_EQ(-0.15, last_stamped_msg_->twist.angular.x);
  EXPECT_DOUBLE_EQ(-0.25, last_stamped_msg_->twist.angular.y);
  EXPECT_DOUBLE_EQ(-0.35, last_stamped_msg_->twist.angular.z);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}