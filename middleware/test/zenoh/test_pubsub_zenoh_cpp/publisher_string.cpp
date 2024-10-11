#include <iostream>
#include <string>
#include <chrono>
#include <queue>
#include <thread>
#include "zenoh_data_publisher.hpp"
#include "zenoh_data_subscriber.hpp"
#include "zenoh_session.hpp"
using namespace std;
using namespace chrono_literals;

const string PUB_KEY = "topic/str";
const string SUB_KEY = "topic/get";
const int HZ = 1000;
const bool USE_SINGLETON = true;
// const bool SINGLE_PC = false;


class ZenohDataPublisher
{
public:
  ZenohDataPublisher(int hz_i):
    zenoh_publisher(),
    zenoh_subscriber(),
    zenoh_session(ZenohSession::get_instance()),
    hz(hz_i)
  {
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
    zenoh_publisher.zenoh_publisher(count_);
    // const char data = count_+ '0';
    // if (!SINGLE_PC)
    // {
    //   zenoh_publisher.zenoh_publisher(count_);
    // }
    // else
    // {
    //   zenoh_publisher.zenoh_publisher_local(&data);
    // }
  }

  void data_sub()
  {
    zenoh_subscriber.zenoh_subscriber(SUB_KEY);
  }

  double milli_sec;
  double duration_sum = 0.0;
  int count_ = 0;
  int sub_count_ = 0;

private:
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

  const int hz;
  queue<pair<int, chrono::high_resolution_clock::time_point>> data_timestamp_queue_;
  IntZenohPub zenoh_publisher;
  IntZenohSub zenoh_subscriber;
  ZenohSession& zenoh_session;
};

int main(int argc, char **argv)
{
  ZenohDataPublisher zdp(HZ);
  auto next_call = chrono::high_resolution_clock::now();
  auto interval = chrono::milliseconds(static_cast<int>(zdp.milli_sec * 1000));
  auto five_mins = chrono::system_clock::now() + chrono::minutes(5);

  while (true) {
    if (chrono::system_clock::now() > five_mins) {
      cout << "pass 5 minutes." << endl;
      cout << "Published Data: " << zdp.count_ << endl;
      cout << "Delay Time Avg: " << (zdp.duration_sum / zdp.sub_count_) << endl;
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