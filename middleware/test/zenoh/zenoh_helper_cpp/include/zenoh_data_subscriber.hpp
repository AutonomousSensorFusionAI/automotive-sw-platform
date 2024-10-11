#ifndef ZENOH_DATA_SUBSCRIBER_HPP
#define ZENOH_DATA_SUBSCRIBER_HPP

#include <string>
#include <functional>
#include <memory>
#include "zenoh.hxx"

using namespace zenoh;
using namespace std;

class IntZenohSub
{
public:
  IntZenohSub();
  
  void init_zenoh_session(const string *configfile);
  void set_session(shared_ptr<Session> opened_session);
  void set_handler(const function<void(const string_view&, const string_view&)>& handler);
  void zenoh_subscriber(const string &key);
  void zenoh_subscriber(const string &key, SubscriberOptions &qos);

private:
  void zenoh_callback(const Sample &sample);
  
  shared_ptr<zenohc::Session> session;
  optional<zenohc::Subscriber> zenoh_sub;
  function<void(const string_view&, const string_view&)> callback_handler;
};

#endif // ZENOH_DATA_SUBSCRIBER_HPP