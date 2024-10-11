#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

using namespace std;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber"), count_(0)
  {
    // subscription_ = this->create_subscription<test_pubsub::msg::Test>(
    //   "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_ = image_transport::create_subscription(
        this, "image_topic",
        std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1),
        "raw");

    rclcpp::on_shutdown(std::bind(&MinimalSubscriber::on_shutdown_callback, this));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    count_++;
    rclcpp::Time sub_time = this->now();
    rclcpp::Time pub_time = msg->header.stamp;

    rclcpp::Duration duration = sub_time - pub_time;
    string duration_str = std::to_string(duration.seconds()) + "." + std::to_string(duration.nanoseconds());

    long int msg_count = std::stoi(msg->header.frame_id);
    if (count_ != msg_count)
    {
      // Publisher가 새로 declare 되었을 상황 가정.
      if (msg_count >= count_)
      {
        count_missing = count_missing + msg_count - count_;
      }
      else
      {
        count_ = 1;
        count_missing = msg_count - count_;
        duration_avg = 0;
      }
      count_ = msg_count;
    }
  }
  void on_shutdown_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown initiated.");
    RCLCPP_INFO(this->get_logger(), "Count: %ld", count_);
    RCLCPP_INFO(this->get_logger(), "Missing count: %ld", count_missing);
  }
  image_transport::Subscriber subscription_;
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
