#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace std::chrono_literals;
const int PIC = 0;

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

    publisher_ = image_transport::create_publisher(this, "image_topic");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(milli_sec), std::bind(&MinimalPublisher::timer_callback, this));

    init_image();
  }

private:
  void init_image()
  {
    string *ROS_WS;

    string package_share_directory = ament_index_cpp::get_package_share_directory("test_pubsub");
    string delimiter = "/install";
    size_t pos = package_share_directory.find(delimiter);
    
    if (pos != string::npos)
    {
      ROS_WS = new string(package_share_directory.substr(0, pos));
      RCLCPP_DEBUG(this->get_logger(), "%s", ROS_WS->c_str());
    }
    else
    {
      ROS_WS = nullptr;
    }

    if (ROS_WS == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Error: can not fine ROS Workspace. Check Package Name == test_pubsub?");
      return;
    }

    const string DIR_PATH = *ROS_WS + "/src/automotive-sw-platform/middleware/test/ros2/test_pubsub/src/image_data/";
    // RCLCPP_INFO(this->get_logger(), "Image path: %s", DIR_PATH.c_str());

    string IMG_PATH;
    if (PIC == 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 5MB");
      IMG_PATH = DIR_PATH + "image_5MB.png";
    }
    else if (PIC == 1)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 1MB");
      IMG_PATH = DIR_PATH + "image_1MB.png";
    }
    else if (PIC == 2)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 0.1MB");
      IMG_PATH = DIR_PATH + "image_100KB.png";
    }
    else if (PIC == 3)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 0.01MB");
      IMG_PATH = DIR_PATH + "image_10KB.png";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Check image file number, again");
    }

    image_ = cv::imread(IMG_PATH, cv::IMREAD_COLOR);

    if (image_.empty()) // error
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image! Check the Path to see if there are images");
      return;
    }
  }
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

    if (!image_.empty())
    {
      // Publish loaded image
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
      msg->header.stamp = now;
      msg->header.frame_id = std::to_string(++count_);
      publisher_.publish(msg);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "No image or pixel data to publish");
      return;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher publisher_;
  rclcpp::Time start_time;
  size_t count_;
  rclcpp::Duration five_minutes_;
  cv::Mat image_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
