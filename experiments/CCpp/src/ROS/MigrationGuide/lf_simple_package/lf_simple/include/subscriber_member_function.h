#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber();

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
