#include <iostream>
#include <fstream>
// #include <nlohmann/json.hpp>
#include "include/zenoh_data_subscriber.hpp"
// using json = nlohmann::json;


IntZenohSub::IntZenohSub()
{
  callback_handler = nullptr;
}
  
void IntZenohSub::init_zenoh_session(const string *configfile)
{
  try
  {
    Config config;
    if (configfile)
    {
      config = expect(config_from_file(configfile->c_str()));
    }
    session = make_shared<Session>(expect<Session>(open(move(config))));
  }
  catch (ErrorMessage e)
  {
    cout << "Error: " << e.as_string_view() << endl;
  }
}

void IntZenohSub::set_session(shared_ptr<Session> opened_session)
{
  session = opened_session;
}

void IntZenohSub::set_handler(const function<void(const string_view&, const string_view&)>& handler) {
  callback_handler = handler;
}

void IntZenohSub::zenoh_callback(const Sample &sample)
{
  string_view key_expr = sample.get_keyexpr().as_string_view();
  string_view payload = sample.get_payload().as_string_view();
  // cout << "Received: " << key_expr << ": " << payload << endl;

  if (callback_handler)
  {
    callback_handler(key_expr, payload);
  }
}

void IntZenohSub::zenoh_subscriber(const string &key)
{
  zenoh_sub = expect<Subscriber>(session->declare_subscriber(key.c_str(), bind(&IntZenohSub::zenoh_callback, this, placeholders::_1)));
}

void IntZenohSub::zenoh_subscriber(const string &key, SubscriberOptions &qos)
{
  zenoh_sub = expect<Subscriber>(session->declare_subscriber(key.c_str(), bind(&IntZenohSub::zenoh_callback, this, placeholders::_1), qos));
}