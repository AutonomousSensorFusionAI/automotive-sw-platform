#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <test_pubsub/msg/test.hpp>

using namespace std;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber"), count_(0)
  {
    subscription_ = this->create_subscription<test_pubsub::msg::Test>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    rclcpp::on_shutdown(std::bind(&MinimalSubscriber::on_shutdown_callback, this));
  }

private:
  void topic_callback(const test_pubsub::msg::Test &msg)
  {
    count_++;
    rclcpp::Time sub_time = this->now();
    rclcpp::Time pub_time = msg.header.stamp;

    rclcpp::Duration duration = sub_time - pub_time;
    string duration_str = std::to_string(duration.seconds()) + "." + std::to_string(duration.nanoseconds());

    if (count_ != msg.data)
    {
      if (msg.data >= count_)
      {
        count_missing = count_missing + msg.data - count_;
      }
      // Publisher가 새로 declare 되었을 상황 가정.
      else
      {
        count_ = 1;
        count_missing = msg.data - count_;
        duration_avg = 0;
      }
      count_ = msg.data;
    }
  }
  void on_shutdown_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown initiated.");
    RCLCPP_INFO(this->get_logger(), "Count: %ld", count_);
    RCLCPP_INFO(this->get_logger(), "Missing count: %ld", count_missing);
  }

  rclcpp::Subscription<test_pubsub::msg::Test>::SharedPtr subscription_;
  long int count_;
  long int count_missing = 0;
  double duration_avg = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
