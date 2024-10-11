#include <stdlib.h> 
#include <iostream>
#include <string>
#include <chrono>
#include <queue>
#include <thread>
#include <opencv2/opencv.hpp>
#include "zenoh.hxx"
#include "zenoh_data_publisher.hpp"
#include "zenoh_data_subscriber.hpp"
#include "zenoh_session.hpp"
#include "image_transmission.pb.h"

using namespace std;
using namespace chrono_literals;
using namespace zenoh;

const string PUB_KEY = "topic/image";
const string SUB_KEY = "topic/get";
const int TOPIC_HZ = 30;
const int PIC = 2;
const bool USE_SINGLETON = true;
// const bool SINGLE_PC = false;


class ZenohDataPublisher
{
public:
  ZenohDataPublisher(int hz_i, int pic_i):
    zenoh_publisher(),
    zenoh_subscriber(),
    zenoh_session(ZenohSession::get_instance()),
    hz(hz_i),
    pic(pic_i)
  {
    init_image();

    milli_sec = 1.0 / hz;
    if (!USE_SINGLETON)
    {
      cout << "Open 'each' Sessions " << endl;
      const string *configfile = nullptr;
      zenoh_publisher.init_zenoh_session(configfile, PUB_KEY);
      zenoh_subscriber.init_zenoh_session(configfile);
    }
    else
    {
      cout << "Using Singleton Pattern Session" << endl;
      auto local_session = zenoh_session.get_local_session();
      zenoh_publisher.set_session(local_session);
      zenoh_publisher.set_key(PUB_KEY);
      zenoh_subscriber.set_session(local_session);
      // if (SINGLE_PC)
      // {
      //   auto shm_manager = zenoh_session.get_shm_manager();
      //   zenoh_publisher.set_shm_manager(shm_manager);
      // }
    }
    zenoh_subscriber.set_handler(bind(&ZenohDataPublisher::data_handler, this, placeholders::_1, placeholders::_2));

    data_sub();
  }

  void data_pub()
  {
    auto pub_time = chrono::high_resolution_clock::now();
    count_++;
    data_timestamp_queue_.emplace(count_, pub_time);
    vector<uint8_t> data = serialize_img(count_, image_);
    BytesView img_data = BytesView(data.data(), data.size());
    zenoh_publisher.zenoh_publisher(img_data);
    // if (!SINGLE_PC)
    // {
    //   zenoh_publisher.zenoh_publisher(img_data);
    // }
    // else
    // {
    //   zenoh_publisher.zenoh_publisher_local(img_data);
    // }
  }

  void data_sub()
  {
    zenoh_subscriber.zenoh_subscriber(SUB_KEY);
  }

  double milli_sec;
  double duration_sum = 0.0;
  double serial_duration_sum = 0.0;
  int count_ = 0;
  int sub_count_ = 0;

private:
  void init_image()
  {
    if (HOME == nullptr) {
      cerr << "Error: HOME environment variable is not set." << endl;
      exit(1);
    }
    const string DIR_PATH = string(HOME) + "/automotive-sw-platform/middleware/test/zenoh/image_data/";
    
    string IMG_PATH;
    if (pic == 0)
    {
      IMG_PATH = DIR_PATH + "nara.png"; // 2448x2048 5.5MB
    }
    else if (pic == 1)
    {
      IMG_PATH = DIR_PATH + "nara_50.png"; // 1224x1024 1.8 MB
    }
    else
    {
      IMG_PATH = DIR_PATH + "nara_25.png"; // 612x512 0.5MB
    }

    image_ = cv::imread(IMG_PATH, cv::IMREAD_COLOR);
    if (image_.empty()) // error
    {
      cerr << "Failed to load image! Check the Path to see if there are images: " << IMG_PATH.c_str() << endl;
      exit(1);
    }
  }

  vector<uint8_t> serialize_img(int count_, const cv::Mat& image)
  {
    auto start_time = chrono::high_resolution_clock::now();
    size_t image_size = image.total() * image.elemSize();

    ImageTransmission im_trans;
    im_trans.set_image_num(count_);
    im_trans.set_image_data(image.data, image_size);
    
    size_t trans_size = im_trans.ByteSizeLong();
    vector<uint8_t> buffer(trans_size);

    if (!im_trans.SerializeToArray(buffer.data(), trans_size)) {
      cerr << "Failed to serialize ImageMessage" << endl;
      exit(1);
    }
    auto trans_time = chrono::high_resolution_clock::now();
    auto serial_duration = chrono::duration<double>(trans_time-start_time).count();
    serial_duration_sum += serial_duration;
    return buffer;
  }

  void data_handler(const string_view &key_expr, const string_view &payload)
  {
    auto sub_time = chrono::high_resolution_clock::now();
    int value = stoi(payload.data());

    if (data_timestamp_queue_.empty()) return;
    auto [pub_count_, pub_time] = data_timestamp_queue_.front();

    if (pub_count_ > value) return;
    sub_count_++;

    while (!data_timestamp_queue_.empty() && (pub_count_ < value)) {
      data_timestamp_queue_.pop();
      if (!data_timestamp_queue_.empty()) {
        tie(pub_count_, pub_time) = data_timestamp_queue_.front();
      }
    }

    if (!data_timestamp_queue_.empty() && (pub_count_ == value)) {
      auto send_receive_send = chrono::duration<double>(sub_time - pub_time).count();
      duration_sum += send_receive_send;
      data_timestamp_queue_.pop();
    }
  }

  const char* HOME = getenv("HOME");
  const int hz;
  const int pic;
  cv::Mat image_;
  queue<pair<int, chrono::high_resolution_clock::time_point>> data_timestamp_queue_;
  IntZenohPub zenoh_publisher;
  IntZenohSub zenoh_subscriber;
  ZenohSession& zenoh_session;
};

int main(int argc, char **argv)
{
  ZenohDataPublisher zdp(TOPIC_HZ, PIC);
  auto next_call = chrono::high_resolution_clock::now();
  auto interval = chrono::milliseconds(static_cast<int>(zdp.milli_sec * 1000));
  auto five_mins = chrono::system_clock::now() + chrono::minutes(5);

  while (true) {
    if (chrono::system_clock::now() > five_mins) {
      cout << "pass 5 minutes." << endl;
      cout << "Published Data: " << zdp.count_ << endl;
      cout << "Delay Time Avg: " << (zdp.duration_sum / zdp.sub_count_) << endl;
      cout << "Serialization Time Avg: " << (zdp.serial_duration_sum / zdp.count_) << endl;
      break;
    }

    next_call += interval;
    zdp.data_pub();

    auto sleep_time = next_call - chrono::high_resolution_clock::now();
    if (sleep_time.count() > 0) {
      this_thread::sleep_for(sleep_time); // Sleep for the remaining interval time
    } else {
      // Skip if the time exceeded (could degrade performance)
      next_call = chrono::high_resolution_clock::now() + interval;
    }
  }

  return 0;
}