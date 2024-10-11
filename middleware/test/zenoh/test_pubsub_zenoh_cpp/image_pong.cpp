#include <stdlib.h>
#include <cmath> 
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <queue>
#include <thread>
#include <csignal>
#include <tuple>
#include <opencv2/opencv.hpp>
#include "zenoh.hxx"
#include "zenoh_data_publisher.hpp"
#include "zenoh_data_subscriber.hpp"
#include "zenoh_session.hpp"
#include "image_transmission.pb.h"

using namespace std;
using namespace chrono_literals;
using namespace zenoh;

const string PUB_KEY = "topic/image_pong";
const string SUB_KEY = "topic/image_ping";
const int PIC = 0; // 0: 5MB, 1: 1MB, 2: 100kB, 3: 10kB
const bool USE_SINGLETON = true;
const double GBE = 1.0; // GbE
const int TEST_COUNT = 1000;

class ZenohDataPublisher
{
public:
  ZenohDataPublisher(int argc, char** argv):
    zenoh_publisher(),
    zenoh_subscriber(),
    zenoh_session(ZenohSession::get_instance())
  {
    int pic, priority, congestion, reliability;
    tie(pic, priority, congestion, reliability) = get_param(argc, argv);
    
    cout << "pic: " << pic << endl;
    init_image(pic);

    PublisherOptions pub_options;
    SubscriberOptions sub_options;
    init_qos(pub_options, sub_options, priority, congestion, reliability);

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
      zenoh_publisher.set_key(PUB_KEY, pub_options);
      zenoh_subscriber.set_session(local_session);
    }
    zenoh_subscriber.set_handler(bind(&ZenohDataPublisher::data_handler, this, placeholders::_1, placeholders::_2));

    data_sub(sub_options);
  }

  void data_pub(int count_)
  {
    vector<uint8_t> data = serialize_img(count_);
    BytesView img_data = BytesView(data.data(), data.size());
    zenoh_publisher.zenoh_publisher(img_data);
    // cout << "Pub count: " << count_ << endl; // For Debug
  }

  void data_sub(SubscriberOptions &sub_options)
  {
    zenoh_subscriber.zenoh_subscriber(SUB_KEY, sub_options);
  }

  int publish_count;
  double duration = 0.0;
  double serial_duration_sum = 0.0;
  double deserial_duration_sum = 0.0;
  chrono::high_resolution_clock::time_point start_time;
  chrono::high_resolution_clock::time_point last_time;

private:
  tuple<int, int, int, int> get_param(int argc, char** argv)
  {
    int pic = PIC;
    int priority = -1;
    int congestion = -1;
    int reliability = -1;

    for (int i = 1; i < argc; i += 2)
    {
      if (i + 1 < argc) {
        string key = argv[i];
        string value = argv[i + 1];

        if (key == "-pic") {
          istringstream iss(value);
          iss >> pic;
        }
        else if (key == "-priority")
        {
          istringstream iss(value);
          iss >> priority;
        }
        else if (key == "-congestion")
        {
          istringstream iss(value);
          iss >> congestion;
        }
        else if (key == "-reliability")
        {
          istringstream iss(value);
          iss >> reliability;
        }
      }
    }

    return make_tuple(pic, priority, congestion, reliability);
  }

  void init_image(int pic)
  {
    if (HOME == nullptr) {
      cerr << "Error: HOME environment variable is not set." << endl;
      exit(1);
    }
    const string DIR_PATH = string(HOME) + "/automotive-sw-platform/middleware/test/zenoh/image_data/";
    
    string IMG_PATH;
    double max_pub;
    if (pic == 0)
    {
      cout << "TEST image 5MB" << endl;
      IMG_PATH = DIR_PATH + "image_5MB.png";
      max_pub = (((GBE * 1000) / 8) / 5);
      width = 1122;
      height = 1122;
    }
    else if (pic == 1)
    {
      cout << "TEST image 1MB" << endl;
      IMG_PATH = DIR_PATH + "image_1MB.png";
      max_pub = (((GBE * 1000) / 8) / 1);
      width = 502;
      height = 502;
    }
    else if (pic == 2)
    {
      cout << "TEST image 0.1MB" << endl;
      IMG_PATH = DIR_PATH + "image_100KB.png";
      max_pub = (((GBE * 1000) / 8) / 0.1);
      width = 184;
      height = 184;
    }
    else if (pic == 3)
    {
      cout << "TEST image 0.01MB" << endl;
      IMG_PATH = DIR_PATH + "image_10KB.png";
      max_pub = (((GBE * 1000) / 8) / 0.01);
      width = 58;
      height = 58;
    }
    else
    {
      throw runtime_error("Check image file number, again");
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

    ifstream file(IMG_PATH, ios::binary | ios::ate);
    if (!file.is_open()) {
      cerr << "파일을 열 수 없습니다." << endl;
      exit(1);
    }
    // 파일 크기 구하기
    streampos file_size = file.tellg();
    // cout << "파일 크기: " << file_size << " bytes" << endl;
    file.seekg(0, ios::beg); // 파일 포인터를 파일의 시작으로 이동

    image_buffer = new vector<char>(file_size);

    file.read(image_buffer->data(), file_size);
    file.close(); // 파일 닫기
    
    // cout << "버퍼의 크기: " << image_buffer->size() << " bytes" << endl;
    if (image_buffer->empty()) // error
    {
      cerr << "Failed to load image! Check the Path to see if there are images: " << IMG_PATH.c_str() << endl;
      exit(1);
    }
  }

  void init_qos(PublisherOptions &pub_options, SubscriberOptions &sub_options, int priority, int congestion, int reliability)
  {
    const array<Priority, 7> priorities = {
      Z_PRIORITY_REAL_TIME,
      Z_PRIORITY_INTERACTIVE_HIGH,
      Z_PRIORITY_INTERACTIVE_LOW,
      Z_PRIORITY_DATA_HIGH,
      Z_PRIORITY_DATA,
      Z_PRIORITY_DATA_LOW,
      Z_PRIORITY_BACKGROUND
    };

    const array<string, 8> priority_names = {
      "",
      "REAL_TIME",
      "INTERACTIVE_HIGH",
      "INTERACTIVE_LOW",
      "DATA_HIGH",
      "DATA",
      "DATA_LOW",
      "BACKGROUND"
    };

    const array<CongestionControl, 2> congestions = {
      Z_CONGESTION_CONTROL_BLOCK,
      Z_CONGESTION_CONTROL_DROP
    };

    const array<string, 2> congestion_names = {
      "BLOCK",
      "DROP"
    };

    const array<Reliability, 2> reliabilities = {
      Z_RELIABILITY_BEST_EFFORT,
      Z_RELIABILITY_RELIABLE
    };

    const array<string, 2> reliability_names = {
      "BEST_EFFORT",
      "RELIABLE"
    };

    if (priority >= 0 && priority < priorities.size())
    {
      pub_options.set_priority(priorities[priority]);
    }

    if (congestion >= 0 && congestion < congestions.size())
    {
      pub_options.set_congestion_control(congestions[congestion]);
    }

    if (reliability >= 0 && reliability < reliabilities.size())
    {
      sub_options.set_reliability(reliabilities[reliability]);
    }

    cout << "Priority: " << priority_names[pub_options.get_priority()] << ", "
         << "CongestionControl: " << congestion_names[pub_options.get_congestion_control()] << ", "
         << "Reliability: " << reliability_names[sub_options.get_reliability()]
         << endl;
  }

  vector<uint8_t> serialize_img(int count_)
  {
    auto ser_start_time = chrono::high_resolution_clock::now();
    size_t image_size = image_buffer->size();

    ImageTransmission im_trans;
    im_trans.set_image_num(count_);
    im_trans.set_width(width);
    im_trans.set_height(height);
    im_trans.set_channels(3);
    im_trans.set_image_data(image_buffer->data(), image_size);
    
    size_t trans_size = im_trans.ByteSizeLong();
    vector<uint8_t> proto_buffer(trans_size);

    if (!im_trans.SerializeToArray(proto_buffer.data(), trans_size)) {
      cerr << "Failed to serialize ImageMessage" << endl;
      exit(1);
    }
    auto trans_time = chrono::high_resolution_clock::now();
    auto serial_duration = chrono::duration<double>(trans_time-ser_start_time).count();
    serial_duration_sum += serial_duration;
    return proto_buffer;
  }

  int deserialize_img(const string_view &payload)
  {
    auto deser_start_time = chrono::high_resolution_clock::now();
    ImageTransmission trans_data;
    if (!trans_data.ParseFromArray(payload.data(), payload.size())) {
      cerr << "Failed to parse ImageMessage: " << trans_data.InitializationErrorString() << endl;
      return 0;
    }
    auto trans_time = chrono::high_resolution_clock::now();
    auto deserial_duration = chrono::duration<double>(trans_time-deser_start_time).count();
    deserial_duration_sum += deserial_duration;
    int image_num = trans_data.image_num();
    return image_num;
  }

  void data_handler(const string_view &key_expr, const string_view &payload)
  {
    int data = deserialize_img(payload);
    if (data == -1)
    {
      exit(1);
    }
    data_pub((data+1));
  }

  const char* HOME = getenv("HOME");
  uint32_t width;
  uint32_t height;
  vector<char>* image_buffer;
  IntZenohPub zenoh_publisher;
  IntZenohSub zenoh_subscriber;
  ZenohSession& zenoh_session;
};

ZenohDataPublisher* g_zdp = nullptr;
int test = TEST_COUNT;

void signal_handler() {
  cout << "Set Test Count: " << test << endl;
  if (g_zdp) {
    cout << "Serial Time avg is " << (g_zdp->serial_duration_sum / (g_zdp->publish_count * TEST_COUNT)) << endl;
    cout << "Deserial Time avg is " << (g_zdp->deserial_duration_sum / (g_zdp->publish_count * TEST_COUNT)) << endl;
    cout << endl;
  }
}

int main(int argc, char **argv)
{
  int pic;
  for (int i = 1; i < argc; i += 2)
  {
    if (i + 1 < argc) {
      string key = argv[i];
      string value = argv[i + 1];

      if (key == "-test")
      {
        istringstream iss(value);
        iss >> test;
      }
      else if (key == "-pic") {
        istringstream iss(value);
        iss >> pic;
      }
    }
  }

  cout << "Set Test Count: " << test << ", Pic: " << pic << endl;

  ZenohDataPublisher zdp(argc, argv);
  g_zdp = &zdp;

  atexit(signal_handler);

  try
  {
    while(true)
    {
      this_thread::sleep_for(chrono::microseconds(10)); // 0.00001
    }
  }
  catch (const exception& e)
  {
    cerr << "Exception: " << e.what() << endl;
    return 1;
  }
  return 0;
}