#include "stamp_dude/pose_to_pose_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <thread>

class PoseToPoseStampedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    node_ = std::make_shared<stamp_dude::PoseToPoseStamped>(options);
    
    // Create publisher to send messages to the node
    pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("pose", 10);
    
    // Create subscription to receive stamped messages from the node
    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_stamped", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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
  
  std::shared_ptr<stamp_dude::PoseToPoseStamped> node_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_stamped_msg_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(PoseToPoseStampedTest, InitializationTest)
{
  ASSERT_NE(nullptr, node_);
}

TEST_F(PoseToPoseStampedTest, MessageConversionTest)
{
  auto pose_msg = geometry_msgs::msg::Pose();
  pose_msg.position.x = 1.0;
  pose_msg.position.y = 2.0;
  pose_msg.position.z = 3.0;
  pose_msg.orientation.x = 0.0;
  pose_msg.orientation.y = 0.0;
  pose_msg.orientation.z = 0.0;
  pose_msg.orientation.w = 1.0;
  
  pub_->publish(pose_msg);
  
  // Wait for message to be processed
  auto start_time = std::chrono::steady_clock::now();
  while (!last_stamped_msg_ && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  ASSERT_NE(nullptr, last_stamped_msg_);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, last_stamped_msg_->pose.position.y);
  EXPECT_DOUBLE_EQ(3.0, last_stamped_msg_->pose.position.z);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->pose.orientation.x);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->pose.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->pose.orientation.z);
  EXPECT_DOUBLE_EQ(1.0, last_stamped_msg_->pose.orientation.w);
  
  // Check that timestamp is set
  auto stamp_sec = last_stamped_msg_->header.stamp.sec;
  auto stamp_nanosec = last_stamped_msg_->header.stamp.nanosec;
  EXPECT_GE(stamp_sec, 0);
  EXPECT_GE(stamp_nanosec, 0u);
}

TEST_F(PoseToPoseStampedTest, MultipleMessagesTest)
{
  auto pose_msg1 = geometry_msgs::msg::Pose();
  pose_msg1.position.x = 1.0;
  pose_msg1.position.y = 2.0;
  pose_msg1.position.z = 3.0;
  pose_msg1.orientation.w = 1.0;
  
  pub_->publish(pose_msg1);
  
  // Wait for first message
  auto start_time = std::chrono::steady_clock::now();
  while (!last_stamped_msg_ && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  ASSERT_NE(nullptr, last_stamped_msg_);
  auto first_stamp = last_stamped_msg_->header.stamp;
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  auto pose_msg2 = geometry_msgs::msg::Pose();
  pose_msg2.position.x = 4.0;
  pose_msg2.position.y = 5.0;
  pose_msg2.position.z = 6.0;
  pose_msg2.orientation.x = 0.7071;
  pose_msg2.orientation.y = 0.0;
  pose_msg2.orientation.z = 0.0;
  pose_msg2.orientation.w = 0.7071;
  
  pub_->publish(pose_msg2);
  
  // Wait for second message
  start_time = std::chrono::steady_clock::now();
  while (last_stamped_msg_->pose.position.x != 4.0 && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_once(std::chrono::milliseconds(10));
  }
  
  EXPECT_DOUBLE_EQ(4.0, last_stamped_msg_->pose.position.x);
  EXPECT_DOUBLE_EQ(5.0, last_stamped_msg_->pose.position.y);
  EXPECT_DOUBLE_EQ(6.0, last_stamped_msg_->pose.position.z);
  EXPECT_NEAR(0.7071, last_stamped_msg_->pose.orientation.x, 0.0001);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->pose.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, last_stamped_msg_->pose.orientation.z);
  EXPECT_NEAR(0.7071, last_stamped_msg_->pose.orientation.w, 0.0001);
  
  auto second_stamp = last_stamped_msg_->header.stamp;
  EXPECT_GT(rclcpp::Time(second_stamp), rclcpp::Time(first_stamp));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}