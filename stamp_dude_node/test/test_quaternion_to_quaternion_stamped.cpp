#include "stamp_dude/quaternion_to_quaternion_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include <chrono>
#include <thread>

class QuaternionToQuaternionStampedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    node_ = std::make_shared<stamp_dude::QuaternionToQuaternionStamped>(options);
    
    // Create publisher to send messages to the node
    pub_ = node_->create_publisher<geometry_msgs::msg::Quaternion>("quaternion", 10);
    
    // Create subscription to receive stamped messages from the node
    sub_ = node_->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "quaternion_stamped", 10,
      [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        last_stamped_msg_ = msg;
      });
    
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }
  
  void TearDown() override
  {
    node_.reset();
    executor_.reset();
  }
  
  std::shared_ptr<stamp_dude::QuaternionToQuaternionStamped> node_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_;
  geometry_msgs::msg::QuaternionStamped::SharedPtr last_stamped_msg_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(QuaternionToQuaternionStampedTest, InitializationTest)
{
  ASSERT_NE(nullptr, node_);
}

TEST_F(QuaternionToQuaternionStampedTest, MessageConversionTest)
{
  auto quaternion_msg = geometry_msgs::msg::Quaternion();
  quaternion_msg.x = 0.0;
  quaternion_msg.y = 0.0;
  quaternion_msg.z = 0.0;
  quaternion_msg.w = 1.0;
  
  pub_->publish(quaternion_msg);
  
  // Wait for message to be processed
  auto start_time = std::chrono::steady_clock::now();
  while (!last_stamped_msg_ && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  ASSERT_NE(nullptr, last_stamped_msg_);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->quaternion.x);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->quaternion.y);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->quaternion.z);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->quaternion.w);
  
  // Check that timestamp is set
  auto stamp_sec = last_stamped_msg_->header.stamp.sec;
  auto stamp_nanosec = last_stamped_msg_->header.stamp.nanosec;
  EXPECT_GE(stamp_sec, 0);
  EXPECT_GE(stamp_nanosec, 0u);
}

TEST_F(QuaternionToQuaternionStampedTest, MultipleMessagesTest)
{
  auto quaternion_msg1 = geometry_msgs::msg::Quaternion();
  quaternion_msg1.x = 0.0;
  quaternion_msg1.y = 0.0;
  quaternion_msg1.z = 0.0;
  quaternion_msg1.w = 1.0;
  
  pub_->publish(quaternion_msg1);
  
  // Wait for first message
  auto start_time = std::chrono::steady_clock::now();
  while (!last_stamped_msg_ && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  ASSERT_NE(nullptr, last_stamped_msg_);
  auto first_stamp = last_stamped_msg_->header.stamp;
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  auto quaternion_msg2 = geometry_msgs::msg::Quaternion();
  quaternion_msg2.x = 0.7071;
  quaternion_msg2.y = 0.0;
  quaternion_msg2.z = 0.0;
  quaternion_msg2.w = 0.7071;
  
  pub_->publish(quaternion_msg2);
  
  // Wait for second message
  start_time = std::chrono::steady_clock::now();
  while (last_stamped_msg_->quaternion.x != 0.7071 && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  EXPECT_NEAR(0.7071, last_stamped_msg_->quaternion.x, 0.0001);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->quaternion.y);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->quaternion.z);
  EXPECT_NEAR(0.7071, last_stamped_msg_->quaternion.w, 0.0001);
  
  auto second_stamp = last_stamped_msg_->header.stamp;
  EXPECT_GT(rclcpp::Time(second_stamp), rclcpp::Time(first_stamp));
}

TEST_F(QuaternionToQuaternionStampedTest, NonNormalizedQuaternionTest)
{
  auto quaternion_msg = geometry_msgs::msg::Quaternion();
  quaternion_msg.x = 1.0;
  quaternion_msg.y = 1.0;
  quaternion_msg.z = 1.0;
  quaternion_msg.w = 1.0;
  
  pub_->publish(quaternion_msg);
  
  // Wait for message to be processed
  auto start_time = std::chrono::steady_clock::now();
  while (!last_stamped_msg_ && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  ASSERT_NE(nullptr, last_stamped_msg_);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->quaternion.x);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->quaternion.y);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->quaternion.z);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->quaternion.w);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}