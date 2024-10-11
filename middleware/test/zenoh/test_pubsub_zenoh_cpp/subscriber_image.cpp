#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <opencv2/opencv.hpp>
#include "zenoh_data_publisher.hpp"
#include "zenoh_data_subscriber.hpp"
#include "zenoh_session.hpp"
#include "image_transmission.pb.h"

using namespace std;
using namespace chrono_literals;

const string SUB_KEY = "topic/image";
const string PUB_KEY = "topic/get";
const bool USE_SINGLETON = true;


class ZenohDataSubscriber
{
public:
  ZenohDataSubscriber():
  zenoh_publisher(),
  zenoh_subscriber(),
  zenoh_session(ZenohSession::get_instance())
  {
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
    }
    zenoh_subscriber.set_handler(bind(&ZenohDataSubscriber::data_handler, this, placeholders::_1, placeholders::_2));
    data_sub();
  }

  void data_sub()
  {
    zenoh_subscriber.zenoh_subscriber(SUB_KEY);
  }

  int count_ = 0;
  int count_missing = 0;
  vector<int> missing_data;
  double deserial_duration_sum = 0.0;

private:
  void data_handler(const string_view &key_expr, const string_view &payload)
  {
    count_++;
    auto start_time = chrono::high_resolution_clock::now();
    int data = deserialize_img(payload);
    auto trans_time = chrono::high_resolution_clock::now();
    auto deserial_duration = chrono::duration<double>(trans_time-start_time).count();
    deserial_duration_sum += deserial_duration;
    if (count_ != data)
    {
      count_missing = count_missing + data - count_;
      cout << "missing data is ";
      for (int i = count_; i < data; ++i) {
        cout << (i + 1) << " ";
        missing_data.push_back(i + 1);
      }
      cout << endl;
      count_ = data;
    }

    zenoh_publisher.zenoh_publisher(count_);
  }

  int deserialize_img(const string_view &payload)
  {
    ImageTransmission trans_data;
    if (!trans_data.ParseFromArray(payload.data(), payload.size())) {
      cerr << "Failed to parse ImageMessage: " << trans_data.InitializationErrorString() << endl;
      return 0;
    }
    int image_num = trans_data.image_num();
    return image_num;
  }

  IntZenohPub zenoh_publisher;
  IntZenohSub zenoh_subscriber;
  ZenohSession& zenoh_session;
};

ZenohDataSubscriber* g_zds = nullptr;

void signal_handler(int signal) {
  if (g_zds) {
    cout << "last data: " << g_zds->count_
              << ", missing count: " << g_zds->count_missing
              << ", Deserialization Time Avg: " << (g_zds->deserial_duration_sum / (g_zds->count_-g_zds->count_missing));
    // 손실 데이터가 많을 때는 렉이 심하게 걸려서 일단 주석처리
    // cout << ", missing data is ";
    // for (int missing : g_zds->missing_data) {
    //   cout << missing << " ";
    // }
    cout << endl;
  }
  exit(signal);
}

int main(int argc, char **argv)
{
  ZenohDataSubscriber zds;
  g_zds = &zds;

  signal(SIGINT, signal_handler);

  try
  {
    while(true)
    {
      this_thread::sleep_for(chrono::microseconds(100)); // 0.0001
    }
  }
  catch (const exception& e)
  {
    cerr << "Exception: " << e.what() << endl;
    return 1;
  }
  return 0;
}