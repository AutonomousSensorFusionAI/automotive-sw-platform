#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <test_pubsub/msg/test.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0), five_minutes_(300, 0)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Set topic's hz you want to test.";

    int64_t hz;
    this->declare_parameter("hz", hz, param_desc);
    this->get_parameter("hz", hz);

    int64_t milli_sec = (1000 / hz);

    publisher_ = this->create_publisher<test_pubsub::msg::Test>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(milli_sec), std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    rclcpp::Time now = this->now();
    if (count_ == 0)
    {
      start_time = now;
    }
    rclcpp::Duration duration = now - start_time;
    if (duration >= five_minutes_)
    {
      RCLCPP_INFO(this->get_logger(), "pass 5 minuites. shutdown this node.");
      RCLCPP_INFO(this->get_logger(), "Published data is %zu", count_);
      rclcpp::shutdown();
      return;
    }

    auto message = test_pubsub::msg::Test();
    message.header.stamp = now;
    message.data = ++count_;
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<test_pubsub::msg::Test>::SharedPtr publisher_;
  rclcpp::Time start_time;
  size_t count_;
  rclcpp::Duration five_minutes_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
