#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
using namespace std::chrono_literals;

const string PUB_TOPIC = "topic/image_ping";
const string SUB_TOPIC = "topic/image_pong";
const double GBE = 1.0; // GbE
const int TEST_COUNT = 100;

class MinimalPing : public rclcpp::Node
{
public:
  MinimalPing()
    : Node("minimal_ping"), end_flag(false), test_count(0), publish_count(0), total_duration(0.0), image_msg_(std::make_shared<sensor_msgs::msg::Image>())
  {
    auto pic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    pic_param_desc.description = "Set image number you want to test. (0: 5MB, 1: 1MB, 2: 100kB, 3: 10kB)";

    auto test_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    test_param_desc.description = "Set Number of Tests you want";

    this->declare_parameter("pic", pic, pic_param_desc);
    this->get_parameter("pic", pic);

    this->declare_parameter("test", test, test_param_desc);
    this->get_parameter("test", test);

    RCLCPP_INFO(this->get_logger(), "Pic Number Set: %d, Test Count Set: %d", pic, test);

    init_image();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(PUB_TOPIC, 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      SUB_TOPIC, 10,
      std::bind(&MinimalPing::data_sub, this, std::placeholders::_1));
    
    start_test();
  }
  bool end_flag;
  int publish_count;
  double total_duration;

private:
  void on_shutdown_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown initiated.");
    RCLCPP_INFO(this->get_logger(), "Test Finished. Publish Count: %d", publish_count);
    RCLCPP_INFO(this->get_logger(), "Time avg is %lf", (time_sum / TEST_COUNT));
  }

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

    string IMG_PATH;
    double max_pub;
    if (pic == 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 5MB");
      IMG_PATH = DIR_PATH + "image_5MB.png";
      max_pub = (((GBE * 1000) / 8) / 5);
      width = 1122;
      height = 1122;
    }
    else if (pic == 1)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 1MB");
      IMG_PATH = DIR_PATH + "image_1MB.png";
      max_pub = (((GBE * 1000) / 8) / 1);
      width = 502;
      height = 502;
    }
    else if (pic == 2)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 0.1MB");
      IMG_PATH = DIR_PATH + "image_100KB.png";
      max_pub = (((GBE * 1000) / 8) / 0.1);
      width = 184;
      height = 184;
    }
    else if (pic == 3)
    {
      RCLCPP_DEBUG(this->get_logger(), "TEST image 0.01MB");
      IMG_PATH = DIR_PATH + "image_10KB.png";
      max_pub = (((GBE * 1000) / 8) / 0.01);
      width = 58;
      height = 58;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Check image file number, again");
    }
    max_pub = ceil(max_pub); // 2.5 GbE의 경우 8을 나누면 312.5 이기 때문에 올림.

    if (int(max_pub) % 2 == 0)
    {
      publish_count = int(max_pub);
    }
    else
    {
      publish_count = int(max_pub) + 1;
    }
    RCLCPP_DEBUG(this->get_logger(), "PingPong Count: %d", publish_count);

    ifstream file(IMG_PATH, ios::binary | ios::ate);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open image file");
      return;
    }
    // 파일 크기 구하기
    streampos file_size = file.tellg();
    RCLCPP_DEBUG(this->get_logger(), "Image File Size: %d", file_size);
    file.seekg(0, ios::beg); // 파일 포인터를 파일의 시작으로 이동

    image_buffer = new vector<char>(file_size);

    file.read(image_buffer->data(), file_size);
    file.close(); // 파일 닫기
    
    RCLCPP_DEBUG(this->get_logger(), "Image Buffer Size: %d", image_buffer->size());
    if (image_buffer->empty()) // error
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image! Check the Path to see if there are images");
      return;
    }
  }

  void start_test()
  {
    if (test_count >= TEST_COUNT)
    {
      RCLCPP_INFO(this->get_logger(), "All tests completed.");
      RCLCPP_INFO(this->get_logger(), "Test Finished. Publish Count: %d", publish_count);
      RCLCPP_INFO(this->get_logger(), "Average Time: %lf", (total_duration / TEST_COUNT));
      data_pub(-1);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Starting test %d of %d", test_count + 1, TEST_COUNT);
    start_time = std::chrono::high_resolution_clock::now();
    data_pub(0);
  }

  void data_pub(int count_)
  {
    if (count_ == 0)
    {
      start_time = chrono::high_resolution_clock::now();
    }
    RCLCPP_DEBUG(this->get_logger(), "Ping Pub: %d", count_);
    image_msg_->header.frame_id = to_string(count_);
    image_msg_->height = width;  // PNG 메타데이터에서 추출해야 함
    image_msg_->width = height;   // PNG 메타데이터에서 추출해야 함
    image_msg_->encoding = "png";
    // image_msg_->is_bigendian = false;
    // image_msg_->step = 0;    // PNG의 경우 필요 없음
    // image_msg_->data = image_buffer;
    image_msg_->data.assign(image_buffer->begin(), image_buffer->end());
    publisher_->publish(*image_msg_);
    last_published_count_ = count_;
    current_retries_ = 0;

    // 기존 타이머가 있다면 취소
    if (retry_timer_) {
      retry_timer_->cancel();
    }

    // 새 타이머 설정
    retry_timer_ = this->create_wall_timer(
      RETRY_INTERVAL,
      std::bind(&MinimalPing::retry_publish, this)
    );
  }

  void retry_publish()
  {
    if (current_retries_ < MAX_RETRIES) {
      RCLCPP_WARN(this->get_logger(), "No response received. Retrying publish for count: %d", last_published_count_);
      data_pub(last_published_count_);
      current_retries_++;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Max retries reached for count: %d. Moving to next count.", last_published_count_);
      data_pub(last_published_count_ + 1);
    }
  }

  void data_sub(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    int data = std::stoi(msg->header.frame_id);
    RCLCPP_DEBUG(this->get_logger(), "Ping Sub: %d", data);

    if (retry_timer_) {
      retry_timer_->cancel();
    }


    if ((data+1) == publish_count)
    {
      last_time = chrono::high_resolution_clock::now();
      double duration = chrono::duration<double>(last_time-start_time).count();
      total_duration += duration;
      test_count++;

      start_test();
      return;
    }
    data_pub((data+1));  
  }

  int test_count;
  int64_t pic = 0;
  int64_t test = TEST_COUNT;
  uint32_t width;
  uint32_t height;
  double time_sum;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  sensor_msgs::msg::Image::SharedPtr image_msg_;
  chrono::high_resolution_clock::time_point start_time;
  chrono::high_resolution_clock::time_point last_time;
  vector<char>* image_buffer;
  //for retry
  rclcpp::TimerBase::SharedPtr retry_timer_;
  const std::chrono::milliseconds RETRY_INTERVAL = std::chrono::milliseconds(1000);  // 1초
  const int MAX_RETRIES = 5;
  int current_retries_ = 0;
  uint32_t last_published_count_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPing>());
  rclcpp::shutdown();

  return 0;
}